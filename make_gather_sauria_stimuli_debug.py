#!/usr/bin/env python3
"""
Create a SAURIA+Gather stimulus set from a normal SAURIA stimulus set.

This is intended for the first full-system integration smoke test:
  1) Gather reads dense values from external DRAM and writes SRAMA/IFMAP.
  2) SAURIA controller starts afterwards.

Important: to make the gathered SRAMA contents survive, this script sets the
controller 'keep A' bit by default. This is suitable for a single/tiny tile test
or a smoke test. For a multi-tile convolution, a single gather preload usually
won't reproduce the original golden output unless the controller/gather sequence
is extended per tile.

Robust padding fixes for the current gather FSM/integration:
  * Every AXI beat occupied by CSR comp_idx metadata is zero-cleared before
    writing the meaningful 32-bit words, so padding words cannot contain stale
    DRAM data.
  * By default, one extra zero AXI64 beat is appended after the packed index
    stream and REG_TOTAL_LEN_IDX is increased accordingly. This works around the
    observed missing-last-index/off-by-one behavior in the integrated SAURIA run.
"""
from __future__ import annotations

import argparse
from pathlib import Path
from math import ceil, log2

# Address map
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

AXI_BYTES = 8
ELEM_W = 16
IDX_W = 32
ELEM_BYTES = ELEM_W // 8
IDX_BYTES = IDX_W // 8
NUM_WORDS = 128
N_BLOCKS = 8
WORDS_PER_LINE = AXI_BYTES // ELEM_BYTES
BYTE_SEL_BITS = int(log2(WORDS_PER_LINE))
ADDR_BITS = int(log2(NUM_WORDS))

CTRL_START_ADDR = 0x4000_0000
CTRL_FLAGS_ADDR = 0x4000_0064   # controller args[21]
CTRL_KEEP_A_BIT = 19


def gaddr(off: int) -> int:
    return GATHER_BASE + off


def nbeats(nbytes: int) -> int:
    return ceil(nbytes / AXI_BYTES)


def stim_write(addr: int, data: int) -> list[int]:
    return [data & 0xFFFFFFFF, addr & 0xFFFFFFFF, 1, 0, 0, 0, 0]


def stim_read(addr: int, expected: int = 0) -> list[int]:
    return [0, addr & 0xFFFFFFFF, 0, 1, 0, expected & 0xFFFFFFFF, 0]


def stim_wait_gather() -> list[int]:
    return [0, 0, 0, 0, 2, 0, 0]


def pack_dense_index(linear_index: int, block: int = 0) -> int:
    byte_sel = linear_index % WORDS_PER_LINE
    word_addr = linear_index // WORDS_PER_LINE
    if word_addr >= NUM_WORDS:
        raise ValueError(f"linear_index={linear_index} exceeds gather local NUM_WORDS={NUM_WORDS}")
    if block >= N_BLOCKS:
        raise ValueError(f"block={block} exceeds N_BLOCKS={N_BLOCKS}")
    return (block << (ADDR_BITS + BYTE_SEL_BITS)) | (word_addr << BYTE_SEL_BITS) | byte_sel


def read_stimuli(path: Path) -> list[list[int]]:
    rows = []
    for line in path.read_text().splitlines():
        if not line.strip():
            continue
        vals = [int(x, 16) for x in line.split()]
        if len(vals) != 7:
            raise ValueError(f"Bad GoldenStimuli row: {line}")
        rows.append(vals)
    return rows


def save_stimuli(path: Path, rows: list[list[int]]) -> None:
    with path.open("w") as f:
        for row in rows:
            # keep compact SAURIA-like hex formatting
            f.write(" ".join(f"{x:X}" for x in row) + "\n")


def read_mem(path: Path) -> list[int]:
    return [int(x.strip(), 16) for x in path.read_text().splitlines() if x.strip()]


def save_mem(path: Path, mem: list[int]) -> None:
    with path.open("w") as f:
        for b in mem:
            f.write(f"{b:X}\n")


def write_le(mem: list[int], addr: int, value: int, nbytes: int) -> None:
    need = addr + nbytes
    if need > len(mem):
        mem.extend([0] * (need - len(mem)))
    for i in range(nbytes):
        mem[addr + i] = (value >> (8 * i)) & 0xFF


def zero_range(mem: list[int], addr: int, nbytes: int) -> None:
    need = addr + nbytes
    if need > len(mem):
        mem.extend([0] * (need - len(mem)))
    for i in range(nbytes):
        mem[addr + i] = 0


def patch_mem_with_identity_csr(
    mem: list[int],
    comp_base: int,
    idx_base: int,
    n_values: int,
    comp_total_beats: int,
    idx_total_beats: int,
) -> None:
    # One CSR row with n_values nonzeros.  The full AXI beat windows that the
    # gather will read are cleared first, so padding words cannot contain stale
    # DRAM bytes from the original SAURIA stimulus.
    comp_idx = [0, n_values, 0]
    packed_idx = [pack_dense_index(i, block=0) for i in range(n_values)]

    zero_range(mem, comp_base, comp_total_beats * AXI_BYTES)
    zero_range(mem, idx_base, idx_total_beats * AXI_BYTES)

    for i, v in enumerate(comp_idx):
        write_le(mem, comp_base + i * IDX_BYTES, v, IDX_BYTES)
    for i, v in enumerate(packed_idx):
        write_le(mem, idx_base + i * IDX_BYTES, v, IDX_BYTES)


def build_gather_rows(dense_base: int, comp_base: int, idx_base: int, n_values: int,
                      n_dense_cols: int, debug_reads: int, idx_extra_beats: int) -> list[list[int]]:
    comp_count = 3
    idx_count = n_values
    comp_total_beats = nbeats(comp_count * IDX_BYTES)
    idx_total_beats = nbeats(idx_count * IDX_BYTES) + idx_extra_beats
    rows = [
        stim_write(gaddr(REG_AR_SIZE), 3),
        stim_write(gaddr(REG_AW_SIZE), 3),
        stim_write(gaddr(REG_AR_ADDR_DENSE_MATRIX), dense_base),
        stim_write(gaddr(REG_TOTAL_LEN_DENSE_MATRIX), nbeats(n_dense_cols * ELEM_BYTES)),
        stim_write(gaddr(REG_AR_ADDR_COMP_IDX), comp_base),
        stim_write(gaddr(REG_TOTAL_LEN_COMP_IDX), comp_total_beats),
        stim_write(gaddr(REG_AR_ADDR_IDX), idx_base),
        stim_write(gaddr(REG_TOTAL_LEN_IDX), idx_total_beats),
        stim_write(gaddr(REG_AXI_WR_AW_ADDR_IN), SRAMA_LOCAL),
        stim_write(gaddr(REG_STATUS_REG_N_ROW), n_dense_cols - 1),
        stim_write(gaddr(REG_IS_SPMM), 0),
        stim_write(gaddr(REG_START), 1),
        stim_wait_gather(),
        stim_read(gaddr(REG_DONE), 1),
    ]
    # Optional debug reads without golden checking by default; expected=0 unless user enables later.
    for i in range(debug_reads):
        rows.append(stim_read(SRAMA_DEBUG + 4 * i, 0))
    return rows


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--in-dir", required=True, help="Directory with original GoldenStimuli.txt/initial_dram.txt/gold_dram.txt/tstcfg.txt")
    ap.add_argument("--out-dir", required=True, help="Output directory for patched stimuli")
    ap.add_argument("--dense-base", type=lambda x: int(x, 0), default=0x0)
    ap.add_argument("--comp-base", type=lambda x: int(x, 0), default=0x2000)
    ap.add_argument("--idx-base", type=lambda x: int(x, 0), default=0x3000)
    ap.add_argument("--n-values", type=int, default=340, help="Number of dense FP16 values gathered into SRAMA")
    ap.add_argument("--n-dense-cols", type=int, default=340, help="Dense row/vector logical columns; status_reg_n_row=n_dense_cols-1")
    ap.add_argument("--debug-reads", type=int, default=8)
    ap.add_argument("--idx-extra-beats", type=int, default=1,
                    help="Extra zero AXI64 beats appended to the index stream and included in REG_TOTAL_LEN_IDX. Default=1 works around the observed missing-last-index issue.")
    ap.add_argument("--no-idx-extra-beat", action="store_true",
                    help="Compatibility alias: force --idx-extra-beats 0.")
    ap.add_argument("--no-keep-a", action="store_true", help="Do not set controller keep-A bit")
    ap.add_argument("--gather-only", action="store_true", help="Generate a minimal stimulus file that runs only gather + debug reads, without SAURIA controller start/wait/check. Useful for small 1x64 subsystem tests and compact VCDs.")
    args = ap.parse_args()
    if args.no_idx_extra_beat:
        args.idx_extra_beats = 0
    if args.idx_extra_beats < 0:
        raise ValueError("--idx-extra-beats must be >= 0")

    comp_total_beats = nbeats(3 * IDX_BYTES)
    idx_total_beats = nbeats(args.n_values * IDX_BYTES) + args.idx_extra_beats

    in_dir = Path(args.in_dir)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    def pick(name: str) -> Path:
        p = in_dir / name
        if p.exists():
            return p
        matches = sorted(in_dir.glob(name.replace(".txt", "*.txt")))
        if matches:
            return matches[0]
        raise FileNotFoundError(p)

    golden_path = pick("GoldenStimuli.txt")
    initial_path = pick("initial_dram.txt")
    gold_path = pick("gold_dram.txt")
    tstcfg_path = pick("tstcfg.txt")

    stim = read_stimuli(golden_path)
    init = read_mem(initial_path)
    gold = read_mem(gold_path)

    # Insert CSR metadata into both init and gold so the external DRAM final check is not affected.
    # The patch function clears the full AXI beat windows first; this removes stale DRAM data
    # such as the previously observed 0x3D2BB879 padding word at comp_base+0xC.
    for mem in (init, gold):
        patch_mem_with_identity_csr(
            mem,
            args.comp_base,
            args.idx_base,
            args.n_values,
            comp_total_beats,
            idx_total_beats,
        )

    gather_rows = build_gather_rows(
        dense_base=args.dense_base,
        comp_base=args.comp_base,
        idx_base=args.idx_base,
        n_values=args.n_values,
        n_dense_cols=args.n_dense_cols,
        debug_reads=args.debug_reads,
        idx_extra_beats=args.idx_extra_beats,
    )

    if args.gather_only:
        # Minimal test: execute gather, read SRAMA debug words, then stop at EOF.
        # No SAURIA controller start and no final DRAM region check.
        stim = gather_rows
    else:
        # Set keep-A bit in controller args[21], unless explicitly disabled.
        if not args.no_keep_a:
            patched = False
            for row in stim:
                data, addr, we = row[0], row[1], row[2]
                if we and addr == CTRL_FLAGS_ADDR:
                    row[0] = data | (1 << CTRL_KEEP_A_BIT)
                    patched = True
                    break
            if not patched:
                raise RuntimeError(f"Could not find controller flags register at 0x{CTRL_FLAGS_ADDR:08X}")

        # Insert gather preamble immediately before controller start at 0x40000000.
        start_idx = None
        for i, row in enumerate(stim):
            data, addr, we = row[0], row[1], row[2]
            if we and addr == CTRL_START_ADDR and data == 3:
                start_idx = i
                break
        if start_idx is None:
            raise RuntimeError("Could not find controller start row: data=3 addr=0x40000000 we=1")
        stim = stim[:start_idx] + gather_rows + stim[start_idx:]

    save_stimuli(out_dir / "GoldenStimuli.txt", stim)
    save_mem(out_dir / "initial_dram.txt", init)
    save_mem(out_dir / "gold_dram.txt", gold)
    (out_dir / "tstcfg.txt").write_text(tstcfg_path.read_text())

    print(f"Wrote patched stimuli to {out_dir}")
    if args.gather_only:
        print(f"Generated gather-only stimulus with {len(gather_rows)} rows; SAURIA controller will not be started")
    else:
        print(f"Inserted {len(gather_rows)} gather rows before SAURIA controller start")
    print(f"CSR comp_idx beats: 0x{comp_total_beats:X}; cleared bytes: 0x{args.comp_base:X}..0x{args.comp_base + comp_total_beats * AXI_BYTES - 1:X}")
    print(f"IDX beats: 0x{idx_total_beats:X} ({nbeats(args.n_values * IDX_BYTES):#x} natural + {args.idx_extra_beats} extra); cleared bytes: 0x{args.idx_base:X}..0x{args.idx_base + idx_total_beats * AXI_BYTES - 1:X}")
    if not args.gather_only and not args.no_keep_a:
        print(f"Set controller keep-A bit: 0x{CTRL_FLAGS_ADDR:08X} bit {CTRL_KEEP_A_BIT}")
    if args.gather_only:
        print("NOTE: Gather-only mode validates integrated gather/DRAM/SRAMA paths, not SAURIA compute correctness.")
    else:
        print("NOTE: This is a smoke/full-flow integration stimulus. For multi-tile conv correctness, gather must be sequenced per IFMAP tile or the controller must support a real skip-ifmap-load mode.")


if __name__ == "__main__":
    main()
