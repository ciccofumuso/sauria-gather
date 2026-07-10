#!/usr/bin/env python3
"""
Generate minimal Verilator stimuli for SAURIA + Gather-only test:
  external DRAM -> gather_frontend_axi -> internal SRAMA/IFMAP

Run from anywhere, e.g.:
  python3 generate_gather_only_stimuli.py --test-dir /path/to/sauria/test

Assumptions:
  - DATA_AXI_DATA_WIDTH = 64, so AXI size = 3 and 8 bytes/beat.
  - Gather AXI-Lite base = 0x7000_0000.
  - Gather writes SRAMA using LOCAL internal address 0x0004_0000.
  - Debug reads SRAMA through SAURIA global config address 0x5004_0000.
  - Register map matches gather_frontend_axi.sv discussed in the integration.
  - First test uses SpMV mode: is_spmm = 0.
"""
from __future__ import annotations

import argparse
from pathlib import Path
from math import ceil, log2

# -----------------------------
# Address map
# -----------------------------
GATHER_BASE = 0x7000_0000
SAURIA_BASE = 0x5000_0000
SRAMA_LOCAL = 0x0004_0000
SRAMA_DEBUG = SAURIA_BASE + SRAMA_LOCAL

# Gather register offsets
REG_START                  = 0x00
REG_IS_SPMM                = 0x04
REG_DONE                   = 0x08
REG_AR_SIZE                = 0x0C
REG_AW_SIZE                = 0x10
REG_AR_ADDR_DENSE_MATRIX   = 0x14
REG_TOTAL_LEN_DENSE_MATRIX = 0x18
REG_AR_ADDR_COMP_IDX       = 0x1C
REG_TOTAL_LEN_COMP_IDX     = 0x20
REG_AR_ADDR_IDX            = 0x24
REG_TOTAL_LEN_IDX          = 0x28
REG_AXI_WR_AW_ADDR_IN      = 0x2C
REG_STATUS_REG_N_ROW       = 0x30

# External DRAM layout used by this test
DENSE_BASE    = 0x0000_1000
COMP_IDX_BASE = 0x0000_2000
IDX_BASE      = 0x0000_3000

# Data widths
AXI_BYTES = 8      # DATA_AXI_DATA_WIDTH=64
ELEM_W = 16       # FP16/raw 16-bit dense elements
IDX_W = 32        # CSR indices / packed dense-address indices
ELEM_BYTES = ELEM_W // 8
IDX_BYTES = IDX_W // 8

N_DENSE_COLS = 6
STATUS_REG_N_ROW_VALUE = N_DENSE_COLS - 1

# Internal gather memory geometry. Keep in sync with gather_frontend parameters.
NUM_WORDS = 128
N_BLOCKS = 8
WORDS_PER_LINE = AXI_BYTES // ELEM_BYTES         # 4 FP16 values per 64-bit beat
BYTE_SEL_BITS = int(log2(WORDS_PER_LINE))        # 2
ADDR_BITS = int(log2(NUM_WORDS))                 # 7 for 128
BLOCK_BITS = int(log2(N_BLOCKS))                 # 3 for 8


def gaddr(off: int) -> int:
    return GATHER_BASE + off


def nbeats(nbytes: int) -> int:
    return ceil(nbytes / AXI_BYTES)


def write_le(mem: bytearray, addr: int, value: int, nbytes: int) -> None:
    """Write value to byte-addressed DRAM image, little-endian."""
    for i in range(nbytes):
        mem[addr + i] = (value >> (8 * i)) & 0xFF


def pack_dense_index(linear_index: int, block: int = 0) -> int:
    """
    Pack the index format expected by address_manager.sv:
      [ block ][ word address ][ byte/element select ]
    For FP16 on 64-bit AXI: byte/element select is 2 bits, selecting one of 4 FP16 lanes.
    """
    byte_sel = linear_index % WORDS_PER_LINE
    word_addr = linear_index // WORDS_PER_LINE
    if word_addr >= NUM_WORDS:
        raise ValueError(f"linear_index {linear_index} exceeds NUM_WORDS={NUM_WORDS}")
    if block >= N_BLOCKS:
        raise ValueError(f"block {block} exceeds N_BLOCKS={N_BLOCKS}")
    return (block << (ADDR_BITS + BYTE_SEL_BITS)) | (word_addr << BYTE_SEL_BITS) | byte_sel


def stim_write(addr: int, data: int) -> list[int]:
    # data, address, write_enable, read_enable, wait_mode, expected_read, check_flag
    return [data & 0xFFFFFFFF, addr & 0xFFFFFFFF, 1, 0, 0, 0, 0]


def stim_read(addr: int, expected: int = 0) -> list[int]:
    return [0, addr & 0xFFFFFFFF, 0, 1, 0, expected & 0xFFFFFFFF, 0]


def stim_wait_gather() -> list[int]:
    # wait_mode=2: veri_top.cc waits for gather_interrupt
    return [0, 0, 0, 0, 2, 0, 0]


def stim_finish() -> list[int]:
    # check_flag=1: triggers external DRAM check and ends test if +check_read_values is not used.
    return [0, 0, 0, 0, 0, 0, 1]


def save_hex_matrix(path: Path, rows: list[list[int]]) -> None:
    with path.open("w") as f:
        for row in rows:
            f.write(" ".join(f"{x:08X}" for x in row) + "\n")


def save_hex_bytes(path: Path, mem: bytearray) -> None:
    with path.open("w") as f:
        for b in mem:
            f.write(f"{b:02X}\n")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--test-dir", required=True, help="Path to SAURIA test directory, e.g. /repo/sauria/test")
    ap.add_argument("--mem-size", type=lambda x: int(x, 0), default=0x4000,
                    help="External DRAM image size in bytes. Default: 0x4000")
    args = ap.parse_args()

    test_dir = Path(args.test_dir).resolve()
    stim_dir = test_dir / "stimuli"
    out_dir = test_dir / "outputs"
    stim_dir.mkdir(parents=True, exist_ok=True)
    out_dir.mkdir(parents=True, exist_ok=True)

    # -----------------------------
    # Test data
    # -----------------------------
    # Dense vector B/x stored in external DRAM. Values are raw 16-bit; use recognizable patterns.
    dense_values = [0x1111, 0x2222, 0x3333, 0x4444, 0x5555, 0x6666, 0x7777, 0x8888]

    # One CSR row with 6 nonzeros: row_ptr = [0, 6], plus 0 sentinel for current gather FSM termination.
    # NOTE: if your CU termination changes, adapt this list.
    comp_idx = [0, 3, 0]

    # Gather dense elements in this order: x[0], x[1], x[2], x[3], x[4], x[5]
    col_idx_linear = [5, 0, 2]
    packed_idx = [pack_dense_index(i, block=0) for i in col_idx_linear]

    mem = bytearray(args.mem_size)

    # Write dense FP16/raw values to DRAM, little-endian 16-bit.
    for i, v in enumerate(dense_values):
        write_le(mem, DENSE_BASE + i * ELEM_BYTES, v, ELEM_BYTES)

    # Write compressed row pointers / row metadata, little-endian 32-bit.
    for i, v in enumerate(comp_idx):
        write_le(mem, COMP_IDX_BASE + i * IDX_BYTES, v, IDX_BYTES)

    # Write packed indices, little-endian 32-bit.
    for i, v in enumerate(packed_idx):
        write_le(mem, IDX_BASE + i * IDX_BYTES, v, IDX_BYTES)

    # The gather writes internal SRAMA, not external DRAM. So gold DRAM is identical.
    save_hex_bytes(stim_dir / "initial_dram.txt", mem)
    save_hex_bytes(stim_dir / "gold_dram.txt", mem)

    # External DRAM check region: one byte at address 0, unchanged.
    # tstcfg.txt has exactly three hex lines: dram_startoffs, dram_outoffs, dram_endoffs.
    with (stim_dir / "tstcfg.txt").open("w") as f:
        f.write("00000000\n00000000\n00000000\n")

    # Expected SRAMA output for first 4 gathered FP16 values:
    gathered = [dense_values[i] for i in col_idx_linear]
    exp_low32 = gathered[0] | (gathered[1] << 16)
    exp_high32 = gathered[2] | (gathered[2] << 16)

    rows: list[list[int]] = []
    rows += [
        stim_write(gaddr(REG_AR_SIZE), 3),
        stim_write(gaddr(REG_AW_SIZE), 3),
        stim_write(gaddr(REG_AR_ADDR_DENSE_MATRIX), DENSE_BASE),
        stim_write(gaddr(REG_TOTAL_LEN_DENSE_MATRIX), nbeats(len(dense_values) * ELEM_BYTES)),
        stim_write(gaddr(REG_AR_ADDR_COMP_IDX), COMP_IDX_BASE),
        stim_write(gaddr(REG_TOTAL_LEN_COMP_IDX), nbeats(len(comp_idx) * IDX_BYTES)),
        stim_write(gaddr(REG_AR_ADDR_IDX), IDX_BASE),
        stim_write(gaddr(REG_TOTAL_LEN_IDX), nbeats(len(packed_idx) * IDX_BYTES)),
        # IMPORTANT: local internal address for SRAMA write, not 0x5004_0000.
        stim_write(gaddr(REG_AXI_WR_AW_ADDR_IN), SRAMA_LOCAL),
        # Not used in SpMV mode by address_manager update_n_col path. Keep zero for this first test.
        stim_write(gaddr(REG_STATUS_REG_N_ROW), STATUS_REG_N_ROW_VALUE),
        # SpMV mode.
        stim_write(gaddr(REG_IS_SPMM), 0),
        # Start gather.
        stim_write(gaddr(REG_START), 1),
        # Wait for gather_interrupt through modified veri_top.cc.
        stim_wait_gather(),
        # Read done sticky from gather. Expected 1 if +check_read_values is used.
        stim_read(gaddr(REG_DONE), 1),
        #        # Read first 64-bit SRAMA beat through 32-bit debug AXI-Lite reads.
        stim_read(SRAMA_DEBUG + 0x0, 0x11116666),
        stim_read(SRAMA_DEBUG + 0x4, 0x00003333),
        stim_read(SRAMA_DEBUG + 0x8, 0x00000000),
        stim_read(SRAMA_DEBUG + 0xC, 0x00000000),

        # Finish simulation and run trivial external DRAM check.
        stim_finish(),
    ]
    save_hex_matrix(stim_dir / "GoldenStimuli.txt", rows)

    print(f"Wrote stimuli to: {stim_dir}")
    print("Expected first SRAMA 64-bit beat:")
    print(f"  gathered FP16/raw values = {[hex(x) for x in gathered]}")
    print(f"  debug read 0x{SRAMA_DEBUG:08X} = 0x{exp_low32:08X}")
    print(f"  debug read 0x{SRAMA_DEBUG+4:08X} = 0x{exp_high32:08X}")
    print("Run, for example:")
    print("  cd <repo>/test/verilator")
    print("  ./Test-Sim +debug +check_read_values +max-cycles=1000000")


if __name__ == "__main__":
    main()
