#!/usr/bin/env python3
"""
Proof-oriented SAURIA + gather test.

Sequence:
  1. SAURIA runs normally with a poisoned IFMAP in DRAM.
  2. Gather reads the correct IFMAP from a separate DRAM staging area and
     overwrites SRAM A.
  3. SAURIA starts again with control_regs[21][26] = 1. With the companion
     RTL patch, only the initial A DMA is skipped; B/C, compute and C
     writeback remain on the standard path.
  4. The original golden output is checked.

The input directory must contain vanilla:
GoldenStimuli.txt, initial_dram.txt, gold_dram.txt and tstcfg.txt.
"""
from __future__ import annotations

from argparse import ArgumentParser
from copy import deepcopy
from math import ceil, log2
from pathlib import Path
import json
import re
import shutil
import struct
from typing import Iterable, Sequence

CTRL_START_ADDR = 0x4000_0000
CTRL_INTR_STATUS_ADDR = 0x4000_000C
CTRL_FLAGS_ADDR = 0x4000_0064
CTRL_START_A_ADDR = 0x4000_0058
CTRL_STANDALONE_BIT = 18
CTRL_STANDALONE_KEEP_A_BIT = 19
CTRL_SKIP_INITIAL_A_BIT = 26

GATHER_BASE = 0x7000_0000
REG_START = 0x00
REG_IS_SPMM = 0x04
REG_DONE = 0x08
REG_AR_SIZE = 0x0C
REG_AW_SIZE = 0x10
REG_AR_ADDR_DENSE_MATRIX = 0x14
REG_TOTAL_LEN_DENSE_MATRIX = 0x18
REG_AR_ADDR_COMP_IDX = 0x1C
REG_TOTAL_LEN_COMP_IDX = 0x20
REG_AR_ADDR_IDX = 0x24
REG_TOTAL_LEN_IDX = 0x28
REG_AXI_WR_AW_ADDR_IN = 0x2C
REG_STATUS_REG_N_ROW = 0x30

SRAMA_LOCAL = 0x0004_0000
SRAMA_DEBUG = 0x5004_0000

AXI_BYTES = 8
ELEM_BYTES = 2
IDX_BYTES = 4
NUM_WORDS = 128
N_BLOCKS = 8
WORDS_PER_LINE = AXI_BYTES // ELEM_BYTES
BYTE_SEL_BITS = int(log2(WORDS_PER_LINE))
ADDR_BITS = int(log2(NUM_WORDS))


def parse_int(value: str | int) -> int:
    return value if isinstance(value, int) else int(value, 0)


def nbeats(nbytes: int) -> int:
    return ceil(nbytes / AXI_BYTES)


def align_up(value: int, alignment: int) -> int:
    return (value + alignment - 1) // alignment * alignment


def gaddr(offset: int) -> int:
    return GATHER_BASE + offset


def stim_write(addr: int, data: int) -> list[int]:
    return [data & 0xFFFF_FFFF, addr & 0xFFFF_FFFF, 1, 0, 0, 0, 0]


def stim_read(addr: int, expected: int = 0) -> list[int]:
    return [0, addr & 0xFFFF_FFFF, 0, 1, 0, expected & 0xFFFF_FFFF, 0]


def stim_wait_sauria() -> list[int]:
    return [0, 0, 0, 0, 1, 0, 0]


def stim_wait_gather() -> list[int]:
    return [0, 0, 0, 0, 2, 0, 0]


def read_stimuli(path: Path) -> list[list[int]]:
    rows: list[list[int]] = []
    for lineno, line in enumerate(path.read_text().splitlines(), start=1):
        if not line.strip():
            continue
        vals = [int(tok, 16) for tok in line.split()]
        if len(vals) != 7:
            raise ValueError(f"{path}:{lineno}: expected 7 columns, found {len(vals)}")
        rows.append(vals)
    return rows


def write_stimuli(path: Path, rows: Sequence[Sequence[int]]) -> None:
    with path.open("w") as f:
        for row in rows:
            f.write(" ".join(f"{int(x) & 0xFFFF_FFFF:X}" for x in row) + "\n")


def read_byte_mem(path: Path) -> list[int]:
    return [int(x.strip(), 16) & 0xFF for x in path.read_text().splitlines() if x.strip()]


def write_byte_mem(path: Path, mem: Sequence[int]) -> None:
    with path.open("w") as f:
        for byte in mem:
            f.write(f"{int(byte) & 0xFF:X}\n")


def ensure_len(mem: list[int], size: int) -> None:
    if size > len(mem):
        mem.extend([0] * (size - len(mem)))


def read_bytes(mem: list[int], addr: int, size: int) -> list[int]:
    ensure_len(mem, addr + size)
    return list(mem[addr:addr + size])


def write_bytes(mem: list[int], addr: int, data: Iterable[int]) -> None:
    data = [int(x) & 0xFF for x in data]
    ensure_len(mem, addr + len(data))
    mem[addr:addr + len(data)] = data


def write_le(mem: list[int], addr: int, value: int, size: int) -> None:
    write_bytes(mem, addr, ((value >> (8 * i)) & 0xFF for i in range(size)))


def find_file(directory: Path, name: str) -> Path:
    path = directory / name
    if not path.exists():
        raise FileNotFoundError(path)
    return path


def find_write(rows: Sequence[Sequence[int]], addr: int) -> tuple[int, list[int]]:
    for index, row in enumerate(rows):
        if row[2] and row[1] == addr:
            return index, list(row)
    raise RuntimeError(f"Could not find write to 0x{addr:08X}")


def find_first_start(rows: Sequence[Sequence[int]]) -> int:
    for index, row in enumerate(rows):
        if row[2] and row[1] == CTRL_START_ADDR and (row[0] & 1):
            return index
    raise RuntimeError("Could not find the first controller start")


def find_wait(rows: Sequence[Sequence[int]], start_index: int, mode: int) -> int:
    for index in range(start_index + 1, len(rows)):
        if rows[index][4] == mode:
            return index
    raise RuntimeError(f"Could not find wait mode {mode} after row {start_index}")


def pack_dense_index(linear_index: int, block: int = 0) -> int:
    byte_sel = linear_index % WORDS_PER_LINE
    word_addr = linear_index // WORDS_PER_LINE
    if word_addr >= NUM_WORDS:
        raise ValueError(
            f"index={linear_index} exceeds the single-block capacity "
            f"of {NUM_WORDS * WORDS_PER_LINE} FP16 values"
        )
    if not 0 <= block < N_BLOCKS:
        raise ValueError(f"block={block} is outside 0..{N_BLOCKS - 1}")
    return (block << (ADDR_BITS + BYTE_SEL_BITS)) | (word_addr << BYTE_SEL_BITS) | byte_sel


def patch_identity_metadata(
    initial: list[int],
    gold: list[int],
    *,
    comp_base: int,
    idx_base: int,
    n_values: int,
    idx_extra_beats: int,
) -> dict[str, int]:
    comp_beats = nbeats(3 * IDX_BYTES)
    idx_natural_beats = nbeats(n_values * IDX_BYTES)
    idx_beats = idx_natural_beats + idx_extra_beats

    for mem in (initial, gold):
        write_bytes(mem, comp_base, [0] * (comp_beats * AXI_BYTES))
        write_bytes(mem, idx_base, [0] * (idx_beats * AXI_BYTES))
        for i, value in enumerate((0, n_values, 0)):
            write_le(mem, comp_base + i * IDX_BYTES, value, IDX_BYTES)
        for i in range(n_values):
            write_le(mem, idx_base + i * IDX_BYTES, pack_dense_index(i), IDX_BYTES)

    return {
        "comp_beats": comp_beats,
        "idx_natural_beats": idx_natural_beats,
        "idx_beats": idx_beats,
    }


def build_gather_rows(
    *,
    dense_base: int,
    comp_base: int,
    idx_base: int,
    n_values: int,
    n_dense_cols: int,
    srama_local_addr: int,
    idx_extra_beats: int,
) -> list[list[int]]:
    return [
        stim_write(gaddr(REG_AR_SIZE), 3),
        stim_write(gaddr(REG_AW_SIZE), 3),
        stim_write(gaddr(REG_AR_ADDR_DENSE_MATRIX), dense_base),
        stim_write(gaddr(REG_TOTAL_LEN_DENSE_MATRIX), nbeats(n_dense_cols * ELEM_BYTES)),
        stim_write(gaddr(REG_AR_ADDR_COMP_IDX), comp_base),
        stim_write(gaddr(REG_TOTAL_LEN_COMP_IDX), nbeats(3 * IDX_BYTES)),
        stim_write(gaddr(REG_AR_ADDR_IDX), idx_base),
        stim_write(gaddr(REG_TOTAL_LEN_IDX), nbeats(n_values * IDX_BYTES) + idx_extra_beats),
        stim_write(gaddr(REG_AXI_WR_AW_ADDR_IN), srama_local_addr),
        stim_write(gaddr(REG_STATUS_REG_N_ROW), n_dense_cols - 1),
        stim_write(gaddr(REG_IS_SPMM), 0),
        stim_write(gaddr(REG_START), 1),
        stim_wait_gather(),
        stim_read(gaddr(REG_DONE), expected=1),
    ]


def little_u32_words(data: Sequence[int], count: int) -> list[int]:
    padded = list(data) + [0] * max(0, count * 4 - len(data))
    return [
        sum(padded[4 * i + j] << (8 * j) for j in range(4))
        for i in range(count)
    ]


def make_test(
    *,
    in_dir: Path,
    out_dir: Path,
    n_values: int,
    n_dense_cols: int | None = None,
    ifmap_dram_base: int | None = None,
    dense_stage_base: int = 0x0009_0000,
    poison_fp16: float = 0.0,
    idx_extra_beats: int = 1,
    srama_offset_bytes: int = 0,
    debug_words: int = 4,
) -> dict:
    in_dir = in_dir.resolve()
    out_dir = out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    if n_dense_cols is None:
        n_dense_cols = n_values
    if n_values <= 0 or n_dense_cols < n_values:
        raise ValueError("Require 0 < n_values <= n_dense_cols")

    rows = read_stimuli(find_file(in_dir, "GoldenStimuli.txt"))
    initial = read_byte_mem(find_file(in_dir, "initial_dram.txt"))
    gold = read_byte_mem(find_file(in_dir, "gold_dram.txt"))

    start_index = find_first_start(rows)
    wait_index = find_wait(rows, start_index, mode=1)

    prefix = deepcopy(rows[:start_index])
    # Preserve every vanilla row between START and the SAURIA wait.
    first_run_rows = deepcopy(rows[start_index:wait_index + 1])
    first_start_row = deepcopy(rows[start_index])
    first_wait_row = deepcopy(rows[wait_index])
    post_rows = deepcopy(rows[wait_index + 1:])

    flags_index, flags_row = find_write(prefix, CTRL_FLAGS_ADDR)
    flags_normal = flags_row[0]
    flags_normal &= ~(1 << CTRL_STANDALONE_BIT)
    flags_normal &= ~(1 << CTRL_STANDALONE_KEEP_A_BIT)
    flags_normal &= ~(1 << CTRL_SKIP_INITIAL_A_BIT)
    prefix[flags_index][0] = flags_normal
    flags_skip_a = flags_normal | (1 << CTRL_SKIP_INITIAL_A_BIT)

    if ifmap_dram_base is None:
        _, start_a_row = find_write(prefix, CTRL_START_A_ADDR)
        ifmap_dram_base = start_a_row[0]

    dense_nbytes = n_dense_cols * ELEM_BYTES
    target_ifmap = read_bytes(initial, ifmap_dram_base, dense_nbytes)

    poison_word = list(struct.pack("<e", poison_fp16))
    poison = (poison_word * n_dense_cols)[:dense_nbytes]
    if poison == target_ifmap:
        poison_word = list(struct.pack("<e", 1.0))
        poison = (poison_word * n_dense_cols)[:dense_nbytes]
    if poison == target_ifmap:
        raise RuntimeError("Poison pattern unexpectedly equals target IFMAP")

    # The source used by a normal A DMA remains poisoned. Mirror it in gold
    # because this input range is intentionally modified and never restored.
    write_bytes(initial, ifmap_dram_base, poison)
    write_bytes(gold, ifmap_dram_base, poison)

    dense_stage_end = dense_stage_base + dense_nbytes
    comp_base = align_up(dense_stage_end, 0x1000)
    idx_base = align_up(comp_base + nbeats(3 * IDX_BYTES) * AXI_BYTES, 0x1000)

    write_bytes(initial, dense_stage_base, target_ifmap)
    write_bytes(gold, dense_stage_base, target_ifmap)
    metadata = patch_identity_metadata(
        initial,
        gold,
        comp_base=comp_base,
        idx_base=idx_base,
        n_values=n_values,
        idx_extra_beats=idx_extra_beats,
    )

    gather_rows = build_gather_rows(
        dense_base=dense_stage_base,
        comp_base=comp_base,
        idx_base=idx_base,
        n_values=n_values,
        n_dense_cols=n_dense_cols,
        srama_local_addr=SRAMA_LOCAL + srama_offset_bytes,
        idx_extra_beats=idx_extra_beats,
    )

    expected_debug = little_u32_words(target_ifmap, debug_words)
    debug_rows = [
        stim_read(SRAMA_DEBUG + srama_offset_bytes + 4 * i, expected=value)
        for i, value in enumerate(expected_debug)
    ]

    rows_out = (
        prefix
        + first_run_rows
        + [
            stim_write(CTRL_INTR_STATUS_ADDR, 0x3),
            stim_write(CTRL_START_ADDR, 0x0),
        ]
        + gather_rows
        + debug_rows
        + [
            stim_write(CTRL_FLAGS_ADDR, flags_skip_a),
            stim_write(CTRL_START_ADDR, first_start_row[0]),
            stim_wait_sauria(),
        ]
        + post_rows
    )

    final_check_positions = [i for i, row in enumerate(rows_out) if row[6] != 0]
    controller_starts = [
        i for i, row in enumerate(rows_out)
        if row[2] and row[1] == CTRL_START_ADDR and (row[0] & 1)
    ]
    if len(controller_starts) != 2:
        raise AssertionError(f"Expected two controller starts, found {controller_starts}")
    if final_check_positions and min(final_check_positions) < controller_starts[1]:
        raise AssertionError("A final DRAM check occurs before the second run")

    write_stimuli(out_dir / "GoldenStimuli.txt", rows_out)
    write_byte_mem(out_dir / "initial_dram.txt", initial)
    write_byte_mem(out_dir / "gold_dram.txt", gold)
    shutil.copy2(find_file(in_dir, "tstcfg.txt"), out_dir / "tstcfg.txt")

    manifest = {
        "mode": "poison_A_then_gather_overwrite_then_skip_initial_A",
        "input_dir": str(in_dir),
        "output_dir": str(out_dir),
        "n_values": n_values,
        "n_dense_cols": n_dense_cols,
        "ifmap_dram_base": f"0x{ifmap_dram_base:08X}",
        "ifmap_nbytes": dense_nbytes,
        "poison_fp16": poison_fp16,
        "dense_stage_base": f"0x{dense_stage_base:08X}",
        "comp_base": f"0x{comp_base:08X}",
        "idx_base": f"0x{idx_base:08X}",
        "gather_output": f"0x{SRAMA_LOCAL + srama_offset_bytes:08X}",
        "skip_initial_A_bit": CTRL_SKIP_INITIAL_A_BIT,
        "flags_first_run": f"0x{flags_normal:08X}",
        "flags_second_run": f"0x{flags_skip_a:08X}",
        "expected_srama_debug": {
            f"0x{SRAMA_DEBUG + srama_offset_bytes + 4*i:08X}": f"0x{value:08X}"
            for i, value in enumerate(expected_debug)
        },
        "metadata": metadata,
        "controller_starts": controller_starts,
        "final_check_positions": final_check_positions,
        "interpretation": {
            "success": (
                "Gather overwrote SRAM A and the second run preserved it while "
                "using normal DMA for B/C and final C writeback."
            ),
            "final_mismatch": (
                "The second run reloaded A from the poisoned DRAM source, or "
                "another data-path/configuration error remains."
            ),
        },
    }
    (out_dir / "sram_a_overwrite_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n"
    )
    return manifest


def analyze_log(log_path: Path, manifest_path: Path) -> dict:
    text = log_path.read_text(errors="replace")
    manifest = json.loads(manifest_path.read_text())
    expected = {
        int(addr, 16): int(value, 16)
        for addr, value in manifest["expected_srama_debug"].items()
    }
    read_re = re.compile(r"Read\s+([0-9A-Fa-f]+)\s+from address\s+([0-9A-Fa-f]+)")
    observed: dict[int, int] = {}
    for value_s, addr_s in read_re.findall(text):
        addr = int(addr_s, 16)
        if addr in expected:
            observed[addr] = int(value_s, 16)

    return {
        "success_banner": "SUCCESS!" in text,
        "benchmark_passed": "Benchmark passed with no errors" in text,
        "debug_reads_match": all(observed.get(a) == v for a, v in expected.items()),
        "expected_debug": {hex(k): hex(v) for k, v in expected.items()},
        "observed_debug": {hex(k): hex(v) for k, v in observed.items()},
    }


def main() -> None:
    parser = ArgumentParser()
    parser.add_argument("--in-dir", type=Path)
    parser.add_argument("--out-dir", type=Path)
    parser.add_argument("--n-values", type=int)
    parser.add_argument("--n-dense-cols", type=int)
    parser.add_argument("--ifmap-dram-base", type=parse_int)
    parser.add_argument("--dense-stage-base", type=parse_int, default=0x90000)
    parser.add_argument("--poison-fp16", type=float, default=0.0)
    parser.add_argument("--idx-extra-beats", type=int, default=1)
    parser.add_argument("--srama-offset-bytes", type=parse_int, default=0)
    parser.add_argument("--debug-words", type=int, default=4)
    parser.add_argument("--analyze-log", type=Path)
    parser.add_argument("--manifest", type=Path)
    args = parser.parse_args()

    if args.analyze_log:
        if not args.manifest:
            parser.error("--analyze-log requires --manifest")
        print(json.dumps(analyze_log(args.analyze_log, args.manifest), indent=2))
        return

    if args.in_dir is None or args.out_dir is None or args.n_values is None:
        parser.error("--in-dir, --out-dir and --n-values are required")

    print(json.dumps(make_test(
        in_dir=args.in_dir,
        out_dir=args.out_dir,
        n_values=args.n_values,
        n_dense_cols=args.n_dense_cols,
        ifmap_dram_base=args.ifmap_dram_base,
        dense_stage_base=args.dense_stage_base,
        poison_fp16=args.poison_fp16,
        idx_extra_beats=args.idx_extra_beats,
        srama_offset_bytes=args.srama_offset_bytes,
        debug_words=args.debug_words,
    ), indent=2))


if __name__ == "__main__":
    main()
