#!/usr/bin/env python3
"""
Generate a single-run Gather -> SAURIA test.

Flow:
    1. Gather reads the original dense A tile from a DRAM staging area.
    2. Gather writes the gathered FP16 values into SAURIA SRAM A.
    3. SAURIA starts once in normal DMA mode.
    4. Custom controller flag bit 26 skips only the initial DMA load of A.
    5. DMA still loads B/C, the core consumes gathered A, computes, and writes C.

Important:
    - Controller flag bit 19 is NOT enabled. It is the standalone keep-A bit.
    - Core keep-A must be 0 on the first core start so the double buffer switches
      to the host bank just written by the gather.
"""
from __future__ import annotations

import argparse
import json
import shutil
from dataclasses import dataclass
from math import ceil, log2
from pathlib import Path
from typing import Iterable, Sequence


GATHER_BASE = 0x7000_0000

REG_START                   = 0x00
REG_IS_SPMM                 = 0x04
REG_DONE                    = 0x08
REG_AR_SIZE                 = 0x0C
REG_AW_SIZE                 = 0x10
REG_AR_ADDR_DENSE_MATRIX    = 0x14
REG_TOTAL_LEN_DENSE_MATRIX  = 0x18
REG_AR_ADDR_COMP_IDX        = 0x1C
REG_TOTAL_LEN_COMP_IDX      = 0x20
REG_AR_ADDR_IDX             = 0x24
REG_TOTAL_LEN_IDX           = 0x28
REG_AXI_WR_AW_ADDR_IN       = 0x2C
REG_STATUS_REG_N_ROW        = 0x30

CTRL_START_ADDR = 0x4000_0000
CTRL_FLAGS_ADDR = 0x4000_0064

CTRL_STANDALONE_BIT = 18
CTRL_KEEP_A_BIT = 19
CTRL_KEEP_B_BIT = 20
CTRL_KEEP_C_BIT = 21
CTRL_SKIP_INITIAL_A_BIT = 26

CORE_SRAMA_DEBUG_BASE = 0x5004_0000
GATHER_SRAMA_OUT_BASE = 0x0004_0000

AXI_BYTES = 8
ELEM_BYTES = 2
IDX_BYTES = 4

NUM_WORDS = 128
WORDS_PER_LINE = AXI_BYTES // ELEM_BYTES
BYTE_SEL_BITS = int(log2(WORDS_PER_LINE))
ADDR_BITS = int(log2(NUM_WORDS))

REQUIRED_FILES = (
    "GoldenStimuli.txt",
    "initial_dram.txt",
    "gold_dram.txt",
    "tstcfg.txt",
)


@dataclass(frozen=True)
class Paths:
    golden: Path
    initial: Path
    gold: Path
    tstcfg: Path


def parse_int(value: str | int) -> int:
    return value if isinstance(value, int) else int(value, 0)


def nbeats(nbytes: int) -> int:
    return ceil(nbytes / AXI_BYTES)


def gaddr(offset: int) -> int:
    return GATHER_BASE + offset


def stim_write(address: int, data: int) -> list[int]:
    return [
        data & 0xFFFF_FFFF,
        address & 0xFFFF_FFFF,
        1, 0, 0, 0, 0,
    ]


def stim_read(address: int, expected: int = 0) -> list[int]:
    return [
        0,
        address & 0xFFFF_FFFF,
        0, 1, 0,
        expected & 0xFFFF_FFFF,
        0,
    ]


def stim_wait_gather() -> list[int]:
    return [0, 0, 0, 0, 2, 0, 0]


def find_file(directory: Path, filename: str) -> Path:
    exact = directory / filename
    if exact.exists():
        return exact

    matches = sorted(directory.glob(filename.replace(".txt", "*.txt")))
    if matches:
        return matches[0]

    raise FileNotFoundError(f"Could not find {filename} in {directory}")


def resolve_paths(directory: Path) -> Paths:
    return Paths(
        golden=find_file(directory, "GoldenStimuli.txt"),
        initial=find_file(directory, "initial_dram.txt"),
        gold=find_file(directory, "gold_dram.txt"),
        tstcfg=find_file(directory, "tstcfg.txt"),
    )


def read_stimuli(path: Path) -> list[list[int]]:
    rows: list[list[int]] = []
    for lineno, raw in enumerate(path.read_text().splitlines(), start=1):
        line = raw.strip()
        if not line:
            continue
        values = [int(token, 16) for token in line.split()]
        if len(values) != 7:
            raise ValueError(
                f"Expected 7 columns at {path}:{lineno}, got {len(values)}"
            )
        rows.append(values)
    return rows


def write_stimuli(path: Path, rows: Sequence[Sequence[int]]) -> None:
    with path.open("w") as stream:
        for row in rows:
            stream.write(
                " ".join(f"{int(value) & 0xFFFF_FFFF:X}" for value in row)
                + "\n"
            )


def read_byte_memory(path: Path) -> list[int]:
    return [
        int(line.strip(), 16) & 0xFF
        for line in path.read_text().splitlines()
        if line.strip()
    ]


def write_byte_memory(path: Path, memory: Sequence[int]) -> None:
    with path.open("w") as stream:
        for value in memory:
            stream.write(f"{int(value) & 0xFF:X}\n")


def ensure_size(memory: list[int], size: int) -> None:
    if len(memory) < size:
        memory.extend([0] * (size - len(memory)))


def read_bytes(memory: list[int], address: int, count: int) -> list[int]:
    ensure_size(memory, address + count)
    return list(memory[address:address + count])


def write_bytes(memory: list[int], address: int, values: Iterable[int]) -> None:
    data = [int(value) & 0xFF for value in values]
    ensure_size(memory, address + len(data))
    memory[address:address + len(data)] = data


def write_le(memory: list[int], address: int, value: int, count: int) -> None:
    write_bytes(
        memory,
        address,
        ((value >> (8 * byte)) & 0xFF for byte in range(count)),
    )


def pack_dense_index(linear_index: int, block: int = 0) -> int:
    byte_select = linear_index % WORDS_PER_LINE
    word_address = linear_index // WORDS_PER_LINE

    if word_address >= NUM_WORDS:
        raise ValueError(
            f"n_values exceeds one gather block: index={linear_index}, "
            f"maximum={NUM_WORDS * WORDS_PER_LINE - 1}"
        )

    return (
        (block << (ADDR_BITS + BYTE_SEL_BITS))
        | (word_address << BYTE_SEL_BITS)
        | byte_select
    )


def patch_identity_metadata(
    memory: list[int],
    *,
    comp_base: int,
    idx_base: int,
    n_values: int,
    idx_extra_beats: int,
) -> dict[str, int]:
    comp_beats = nbeats(3 * IDX_BYTES)
    idx_natural_beats = nbeats(n_values * IDX_BYTES)
    idx_beats = idx_natural_beats + idx_extra_beats

    write_bytes(memory, comp_base, [0] * (comp_beats * AXI_BYTES))
    write_bytes(memory, idx_base, [0] * (idx_beats * AXI_BYTES))

    # One CSR row: row_ptr=[0, n_values], followed by the terminating zero
    # expected by the current gather implementation.
    for index, value in enumerate((0, n_values, 0)):
        write_le(memory, comp_base + index * IDX_BYTES, value, IDX_BYTES)

    for index in range(n_values):
        write_le(
            memory,
            idx_base + index * IDX_BYTES,
            pack_dense_index(index),
            IDX_BYTES,
        )

    return {
        "comp_beats": comp_beats,
        "idx_natural_beats": idx_natural_beats,
        "idx_beats": idx_beats,
    }


def build_gather_rows(
    *,
    dense_stage_base: int,
    comp_base: int,
    idx_base: int,
    output_base: int,
    n_values: int,
    n_dense_cols: int,
    idx_extra_beats: int,
) -> list[list[int]]:
    metadata = {
        "comp_beats": nbeats(3 * IDX_BYTES),
        "idx_beats": nbeats(n_values * IDX_BYTES) + idx_extra_beats,
    }

    return [
        stim_write(gaddr(REG_AR_SIZE), 3),
        stim_write(gaddr(REG_AW_SIZE), 3),
        stim_write(gaddr(REG_AR_ADDR_DENSE_MATRIX), dense_stage_base),
        stim_write(
            gaddr(REG_TOTAL_LEN_DENSE_MATRIX),
            nbeats(n_dense_cols * ELEM_BYTES),
        ),
        stim_write(gaddr(REG_AR_ADDR_COMP_IDX), comp_base),
        stim_write(gaddr(REG_TOTAL_LEN_COMP_IDX), metadata["comp_beats"]),
        stim_write(gaddr(REG_AR_ADDR_IDX), idx_base),
        stim_write(gaddr(REG_TOTAL_LEN_IDX), metadata["idx_beats"]),
        stim_write(gaddr(REG_AXI_WR_AW_ADDR_IN), output_base),
        stim_write(gaddr(REG_STATUS_REG_N_ROW), n_dense_cols - 1),
        stim_write(gaddr(REG_IS_SPMM), 0),
        stim_write(gaddr(REG_START), 1),
        stim_wait_gather(),
        stim_read(gaddr(REG_DONE), expected=1),
    ]


def u32_le(data: Sequence[int]) -> int:
    padded = list(data[:4]) + [0] * max(0, 4 - len(data))
    return sum((padded[index] & 0xFF) << (8 * index) for index in range(4))


def patch_controller_flags(rows: list[list[int]]) -> tuple[int, int]:
    matches = [
        row for row in rows
        if row[2] == 1 and row[1] == CTRL_FLAGS_ADDR
    ]
    if len(matches) != 1:
        raise RuntimeError(
            f"Expected exactly one controller flags write at "
            f"0x{CTRL_FLAGS_ADDR:08X}, found {len(matches)}"
        )

    row = matches[0]
    old_flags = row[0]

    # Normal integrated controller mode.
    new_flags = old_flags & ~(1 << CTRL_STANDALONE_BIT)

    # Standalone keep bits must remain disabled. They do not skip DMA A.
    new_flags &= ~(1 << CTRL_KEEP_A_BIT)
    new_flags &= ~(1 << CTRL_KEEP_B_BIT)
    new_flags &= ~(1 << CTRL_KEEP_C_BIT)

    # Custom flag: skip only the first external-DRAM -> SRAM-A DMA.
    new_flags |= 1 << CTRL_SKIP_INITIAL_A_BIT

    row[0] = new_flags & 0xFFFF_FFFF
    return old_flags, new_flags


def count_controller_starts(rows: Sequence[Sequence[int]]) -> list[int]:
    return [
        index
        for index, row in enumerate(rows)
        if row[2] == 1
        and row[1] == CTRL_START_ADDR
        and row[0] == 3
    ]


def generate(
    *,
    in_dir: Path,
    out_dir: Path,
    n_values: int,
    n_dense_cols: int,
    ifmap_dram_base: int,
    dense_stage_base: int,
    comp_base: int,
    idx_base: int,
    gather_output_base: int,
    idx_extra_beats: int,
    poison_original_a: bool,
    debug_words: int,
) -> dict:
    source = resolve_paths(in_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = read_stimuli(source.golden)
    initial_memory = read_byte_memory(source.initial)
    gold_memory = read_byte_memory(source.gold)

    starts_before = count_controller_starts(rows)
    if len(starts_before) != 1:
        raise RuntimeError(
            f"The vanilla input must contain exactly one SAURIA controller "
            f"start; found positions {starts_before}"
        )

    old_flags, new_flags = patch_controller_flags(rows)

    dense_bytes = n_dense_cols * ELEM_BYTES
    gathered_bytes = n_values * ELEM_BYTES
    original_a = read_bytes(initial_memory, ifmap_dram_base, dense_bytes)

    # Stage the real A tile for the gather in both memories. Mirroring the
    # staging/metadata bytes in gold_dram keeps full-memory comparisons stable.
    write_bytes(initial_memory, dense_stage_base, original_a)
    write_bytes(gold_memory, dense_stage_base, original_a)

    metadata_initial = patch_identity_metadata(
        initial_memory,
        comp_base=comp_base,
        idx_base=idx_base,
        n_values=n_values,
        idx_extra_beats=idx_extra_beats,
    )
    metadata_gold = patch_identity_metadata(
        gold_memory,
        comp_base=comp_base,
        idx_base=idx_base,
        n_values=n_values,
        idx_extra_beats=idx_extra_beats,
    )
    assert metadata_initial == metadata_gold

    # Poison the original A DRAM source. A correct run must still pass because
    # SAURIA skips this DMA and consumes the gather-written SRAM-A bank.
    if poison_original_a:
        poison = [0] * gathered_bytes
        write_bytes(initial_memory, ifmap_dram_base, poison)
        write_bytes(gold_memory, ifmap_dram_base, poison)

    gather_rows = build_gather_rows(
        dense_stage_base=dense_stage_base,
        comp_base=comp_base,
        idx_base=idx_base,
        output_base=gather_output_base,
        n_values=n_values,
        n_dense_cols=n_dense_cols,
        idx_extra_beats=idx_extra_beats,
    )

    debug_rows: list[list[int]] = []
    for word_index in range(debug_words):
        byte_offset = word_index * 4
        expected = u32_le(original_a[byte_offset:byte_offset + 4])
        debug_rows.append(
            stim_read(CORE_SRAMA_DEBUG_BASE + byte_offset, expected)
        )

    final_rows = gather_rows + debug_rows + rows
    starts_after = count_controller_starts(final_rows)
    if len(starts_after) != 1:
        raise AssertionError(
            f"Generated test must contain one SAURIA start, got {starts_after}"
        )

    write_stimuli(out_dir / "GoldenStimuli.txt", final_rows)
    write_byte_memory(out_dir / "initial_dram.txt", initial_memory)
    write_byte_memory(out_dir / "gold_dram.txt", gold_memory)
    shutil.copy2(source.tstcfg, out_dir / "tstcfg.txt")

    manifest = {
        "mode": "gather_then_single_sauria_skip_initial_A",
        "input_dir": str(in_dir.resolve()),
        "output_dir": str(out_dir.resolve()),
        "n_values": n_values,
        "n_dense_cols": n_dense_cols,
        "ifmap_dram_base": f"0x{ifmap_dram_base:08X}",
        "dense_stage_base": f"0x{dense_stage_base:08X}",
        "comp_base": f"0x{comp_base:08X}",
        "idx_base": f"0x{idx_base:08X}",
        "gather_output": f"0x{gather_output_base:08X}",
        "poison_original_a": poison_original_a,
        "flags_old": f"0x{old_flags:08X}",
        "flags_new": f"0x{new_flags:08X}",
        "standalone": bool((new_flags >> CTRL_STANDALONE_BIT) & 1),
        "standalone_keep_A": bool((new_flags >> CTRL_KEEP_A_BIT) & 1),
        "skip_initial_A": bool(
            (new_flags >> CTRL_SKIP_INITIAL_A_BIT) & 1
        ),
        "metadata": metadata_initial,
        "controller_starts": starts_after,
        "debug_words": {
            f"0x{CORE_SRAMA_DEBUG_BASE + index * 4:08X}":
            f"0x{u32_le(original_a[index * 4:index * 4 + 4]):08X}"
            for index in range(debug_words)
        },
        "expected_bank_sequence": [
            "reset: select_A=0, host/gather accesses physical A bank 0",
            "gather writes physical A bank 0",
            "initial A DMA is skipped; B and C DMA still run",
            "first core start uses keep_A=0 and toggles select_A to 1",
            "core therefore reads physical A bank 0 written by gather",
        ],
    }

    (out_dir / "gather_then_sauria_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n"
    )
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--in-dir", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--n-values", type=int, required=True)
    parser.add_argument("--n-dense-cols", type=int)
    parser.add_argument("--ifmap-dram-base", type=parse_int, default=0)
    parser.add_argument("--dense-stage-base", type=parse_int, default=0x90000)
    parser.add_argument("--comp-base", type=parse_int, default=0x91000)
    parser.add_argument("--idx-base", type=parse_int, default=0x92000)
    parser.add_argument(
        "--gather-output-base",
        type=parse_int,
        default=GATHER_SRAMA_OUT_BASE,
    )
    parser.add_argument("--idx-extra-beats", type=int, default=1)
    parser.add_argument("--debug-words", type=int, default=4)
    parser.add_argument(
        "--no-poison-original-a",
        action="store_true",
        help="Keep the original A bytes in DRAM instead of replacing them by zero.",
    )
    args = parser.parse_args()

    n_dense_cols = (
        args.n_values
        if args.n_dense_cols is None
        else args.n_dense_cols
    )

    if args.n_values <= 0:
        raise ValueError("--n-values must be positive")
    if n_dense_cols < args.n_values:
        raise ValueError("--n-dense-cols cannot be smaller than --n-values")

    manifest = generate(
        in_dir=args.in_dir,
        out_dir=args.out_dir,
        n_values=args.n_values,
        n_dense_cols=n_dense_cols,
        ifmap_dram_base=args.ifmap_dram_base,
        dense_stage_base=args.dense_stage_base,
        comp_base=args.comp_base,
        idx_base=args.idx_base,
        gather_output_base=args.gather_output_base,
        idx_extra_beats=args.idx_extra_beats,
        poison_original_a=not args.no_poison_original_a,
        debug_words=args.debug_words,
    )
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
