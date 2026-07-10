#!/usr/bin/env python3
"""
Utilities to generate SAURIA + gather DRAM-writeback stimuli from vanilla SAURIA stimuli.

Mode supported here:
    gather reads dense/CSR/idx from external DRAM
    gather writes gathered IFMAP to external DRAM at the same addresses SAURIA will read
    SAURIA then runs normally and uses its own DMA IFMAP preload

This does NOT modify gather_frontend_axi. It assumes the subsystem routes:
    gather_ext_mem.AR/R       -> external DRAM during gather
    gather_sauria_mem.AW/W/B  -> external DRAM during gather
    dma_ext_mem               -> external DRAM when gather is not busy

GoldenStimuli row format:
    data address write_enable read_enable wait_mode expected_read check_flag

Important for the current Verilator testbench:
    check_flag == 1 triggers the final gold_dram comparison. Therefore REG_DONE
    reads generated here use expected_read=1 but check_flag=0.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from math import ceil, log2
from typing import Iterable, Sequence
import csv
import json
import re
import shutil

# -----------------------------
# Address maps / register maps
# -----------------------------
GATHER_BASE = 0x7000_0000

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

CTRL_BASE       = 0x4000_0000
CTRL_START_ADDR = 0x4000_0000
CTRL_FLAGS_ADDR = 0x4000_0064
CTRL_KEEP_A_BIT = 19

AXI_BYTES = 8
ELEM_W = 16
IDX_W = 32
ELEM_BYTES = ELEM_W // 8
IDX_BYTES = IDX_W // 8

# gather address-manager parameters used in the current integration
NUM_WORDS = 128
N_BLOCKS = 8
WORDS_PER_LINE = AXI_BYTES // ELEM_BYTES       # 4 FP16 per AXI64 beat
BYTE_SEL_BITS = int(log2(WORDS_PER_LINE))      # 2
ADDR_BITS = int(log2(NUM_WORDS))               # 7


@dataclass(frozen=True)
class MemRange:
    name: str
    start: int
    end_excl: int

    @property
    def nbytes(self) -> int:
        return max(0, self.end_excl - self.start)

    def overlaps(self, other: "MemRange") -> bool:
        return self.start < other.end_excl and other.start < self.end_excl

    def __str__(self) -> str:
        if self.nbytes == 0:
            return f"{self.name}: <empty>"
        return f"{self.name}: 0x{self.start:X}..0x{self.end_excl - 1:X} ({self.nbytes} bytes)"


def parse_int(x: str | int) -> int:
    if isinstance(x, int):
        return x
    return int(str(x), 0)


def gaddr(off: int) -> int:
    return GATHER_BASE + off


def nbeats(nbytes: int) -> int:
    return ceil(nbytes / AXI_BYTES)


def stim_write(addr: int, data: int) -> list[int]:
    return [data & 0xFFFFFFFF, addr & 0xFFFFFFFF, 1, 0, 0, 0, 0]


def stim_read(addr: int, expected: int = 0, final_check: bool = False) -> list[int]:
    return [0, addr & 0xFFFFFFFF, 0, 1, 0, expected & 0xFFFFFFFF, 1 if final_check else 0]


def stim_wait_gather() -> list[int]:
    return [0, 0, 0, 0, 2, 0, 0]


def pack_dense_index(linear_index: int, block: int = 0) -> int:
    """Pack an FP16 dense vector element index for the gather index stream."""
    byte_sel = linear_index % WORDS_PER_LINE
    word_addr = linear_index // WORDS_PER_LINE
    if word_addr >= NUM_WORDS:
        raise ValueError(
            f"linear_index={linear_index} exceeds one gather block: "
            f"NUM_WORDS={NUM_WORDS}, WORDS_PER_LINE={WORDS_PER_LINE}. "
            "Use fewer n_values per gather start or extend the metadata generator."
        )
    if block >= N_BLOCKS:
        raise ValueError(f"block={block} exceeds N_BLOCKS={N_BLOCKS}")
    return (block << (ADDR_BITS + BYTE_SEL_BITS)) | (word_addr << BYTE_SEL_BITS) | byte_sel


# -----------------------------
# Basic file helpers
# -----------------------------
def read_stimuli(path: Path) -> list[list[int]]:
    rows: list[list[int]] = []
    for lineno, line in enumerate(path.read_text().splitlines(), start=1):
        line = line.strip()
        if not line:
            continue
        vals = [int(tok, 16) for tok in line.split()]
        if len(vals) != 7:
            raise ValueError(f"Bad GoldenStimuli row at {path}:{lineno}: {line}")
        rows.append(vals)
    return rows


def write_stimuli(path: Path, rows: Sequence[Sequence[int]]) -> None:
    with path.open("w") as f:
        for row in rows:
            f.write(" ".join(f"{int(x) & 0xFFFFFFFF:X}" for x in row) + "\n")


def read_byte_mem(path: Path) -> list[int]:
    return [int(x.strip(), 16) & 0xFF for x in path.read_text().splitlines() if x.strip()]


def write_byte_mem(path: Path, mem: Sequence[int]) -> None:
    with path.open("w") as f:
        for b in mem:
            f.write(f"{int(b) & 0xFF:X}\n")


def ensure_len(mem: list[int], size: int) -> None:
    if size > len(mem):
        mem.extend([0] * (size - len(mem)))


def read_bytes(mem: list[int], addr: int, nbytes: int) -> list[int]:
    ensure_len(mem, addr + nbytes)
    return list(mem[addr:addr + nbytes])


def write_bytes(mem: list[int], addr: int, data: Iterable[int]) -> None:
    data = [int(x) & 0xFF for x in data]
    ensure_len(mem, addr + len(data))
    mem[addr:addr + len(data)] = data


def zero_range(mem: list[int], addr: int, nbytes: int) -> None:
    write_bytes(mem, addr, [0] * nbytes)


def write_le(mem: list[int], addr: int, value: int, nbytes: int) -> None:
    ensure_len(mem, addr + nbytes)
    for i in range(nbytes):
        mem[addr + i] = (value >> (8 * i)) & 0xFF


def find_file(directory: Path, name: str) -> Path:
    p = directory / name
    if p.exists():
        return p
    matches = sorted(directory.glob(name.replace(".txt", "*.txt")))
    if matches:
        return matches[0]
    raise FileNotFoundError(f"Could not find {name} in {directory}")


def copy_stimuli_dir(src: Path, dst: Path) -> None:
    dst.mkdir(parents=True, exist_ok=True)
    for name in ["GoldenStimuli.txt", "initial_dram.txt", "gold_dram.txt", "tstcfg.txt"]:
        shutil.copy2(find_file(src, name), dst / name)


# -----------------------------
# SAURIA/gather patch helpers
# -----------------------------
def clear_keep_a(rows: list[list[int]]) -> bool:
    """Clear keep-A so SAURIA uses normal IFMAP DMA preload from external DRAM."""
    patched = False
    for row in rows:
        data, addr, we = row[0], row[1], row[2]
        if we and addr == CTRL_FLAGS_ADDR:
            row[0] = data & ~(1 << CTRL_KEEP_A_BIT)
            patched = True
            break
    return patched


def first_controller_access_index(rows: Sequence[Sequence[int]]) -> int:
    for i, row in enumerate(rows):
        addr, we, re = row[1], row[2], row[3]
        if (we or re) and CTRL_BASE <= addr < CTRL_BASE + 0x10000:
            return i
    return 0


def find_controller_start_index(rows: Sequence[Sequence[int]]) -> int:
    for i, row in enumerate(rows):
        data, addr, we = row[0], row[1], row[2]
        if we and addr == CTRL_START_ADDR and data == 3:
            return i
    raise RuntimeError("Could not find SAURIA start row: data=3 addr=0x40000000 we=1")


def patch_identity_csr_metadata(
    mem: list[int],
    *,
    comp_base: int,
    idx_base: int,
    n_values: int,
    idx_extra_beats: int = 1,
) -> dict[str, int]:
    """Write CSR metadata for one row with identity col_idx [0..n_values-1]."""
    comp_total_beats = nbeats(3 * IDX_BYTES)
    idx_total_beats = nbeats(n_values * IDX_BYTES) + idx_extra_beats

    # Clear full AXI windows first: removes stale padding words.
    zero_range(mem, comp_base, comp_total_beats * AXI_BYTES)
    zero_range(mem, idx_base, idx_total_beats * AXI_BYTES)

    for i, value in enumerate([0, n_values, 0]):
        write_le(mem, comp_base + i * IDX_BYTES, value, IDX_BYTES)
    for i in range(n_values):
        write_le(mem, idx_base + i * IDX_BYTES, pack_dense_index(i), IDX_BYTES)

    return {
        "comp_total_beats": comp_total_beats,
        "idx_total_beats": idx_total_beats,
        "idx_natural_beats": nbeats(n_values * IDX_BYTES),
    }


def build_one_gather_rows(
    *,
    dense_base: int,
    comp_base: int,
    idx_base: int,
    out_base: int,
    n_values: int,
    n_dense_cols: int,
    idx_extra_beats: int = 1,
    check_done_read: bool = False,
) -> list[list[int]]:
    comp_total_beats = nbeats(3 * IDX_BYTES)
    idx_total_beats = nbeats(n_values * IDX_BYTES) + idx_extra_beats

    return [
        stim_write(gaddr(REG_AR_SIZE), 3),
        stim_write(gaddr(REG_AW_SIZE), 3),
        stim_write(gaddr(REG_AR_ADDR_DENSE_MATRIX), dense_base),
        stim_write(gaddr(REG_TOTAL_LEN_DENSE_MATRIX), nbeats(n_dense_cols * ELEM_BYTES)),
        stim_write(gaddr(REG_AR_ADDR_COMP_IDX), comp_base),
        stim_write(gaddr(REG_TOTAL_LEN_COMP_IDX), comp_total_beats),
        stim_write(gaddr(REG_AR_ADDR_IDX), idx_base),
        stim_write(gaddr(REG_TOTAL_LEN_IDX), idx_total_beats),
        stim_write(gaddr(REG_AXI_WR_AW_ADDR_IN), out_base),
        stim_write(gaddr(REG_STATUS_REG_N_ROW), n_dense_cols - 1),
        stim_write(gaddr(REG_IS_SPMM), 0),
        stim_write(gaddr(REG_START), 1),
        stim_wait_gather(),
        # expected_read=1 but final_check=0. In this testbench final_check=1 starts gold_dram compare.
        stim_read(gaddr(REG_DONE), expected=1, final_check=check_done_read),
    ]


def unique_preserve_order(values: Iterable[int]) -> list[int]:
    seen: set[int] = set()
    out: list[int] = []
    for v in values:
        if v not in seen:
            seen.add(v)
            out.append(v)
    return out


def make_gather_dram_writeback_stimuli(
    *,
    in_dir: str | Path,
    out_dir: str | Path,
    tile_out_bases: Sequence[int] | None = None,
    gather_out_base: int = 0,
    n_values: int,
    n_dense_cols: int | None = None,
    dense_stage_base: int = 0x90000,
    dense_stage_stride: int = 0x1000,
    comp_base: int | None = None,
    idx_base: int | None = None,
    idx_extra_beats: int = 1,
    poison_out: bool = True,
    poison_value: int = 0,
    clear_keep_a_bit: bool = True,
    insert_position: str = "prepend",
    deduplicate_tiles: bool = True,
    patch_gold_staging: bool = False,
) -> dict:
    """Generate stimuli where gather writes IFMAP tiles to external DRAM, then SAURIA runs.

    tile_out_bases are the DRAM addresses that SAURIA will later read as IFMAP.
    If tile_out_bases is None, a single tile at gather_out_base is generated.
    """
    in_dir = Path(in_dir)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    if n_dense_cols is None:
        n_dense_cols = n_values
    if n_values <= 0 or n_dense_cols <= 0:
        raise ValueError("n_values and n_dense_cols must be positive")
    if n_values > n_dense_cols:
        raise ValueError("n_values cannot be larger than n_dense_cols for identity CSR metadata")

    tile_out_bases = [gather_out_base] if tile_out_bases is None else [int(x) for x in tile_out_bases]
    if deduplicate_tiles:
        tile_out_bases = unique_preserve_order(tile_out_bases)

    golden_path = find_file(in_dir, "GoldenStimuli.txt")
    init_path = find_file(in_dir, "initial_dram.txt")
    gold_path = find_file(in_dir, "gold_dram.txt")
    tstcfg_path = find_file(in_dir, "tstcfg.txt")

    rows = read_stimuli(golden_path)
    init = read_byte_mem(init_path)
    gold = read_byte_mem(gold_path)

    if clear_keep_a_bit and not clear_keep_a(rows):
        raise RuntimeError(f"Could not find controller flags register at 0x{CTRL_FLAGS_ADDR:08X}")

    dense_nbytes = n_dense_cols * ELEM_BYTES
    gathered_nbytes = n_values * ELEM_BYTES

    # Place metadata after the dense staging area unless explicit addresses are given.
    # The old fixed defaults (comp=0xA0000, idx=0xA1000) overlapped the dense staging
    # range when many tiles were generated: dense_stage_base + 16*0x1000 == 0xA0000.
    def align_up(x: int, a: int = 0x1000) -> int:
        return (x + a - 1) & ~(a - 1)

    num_stage_tiles = max(1, len(tile_out_bases))
    dense_stage_end = dense_stage_base + (num_stage_tiles - 1) * dense_stage_stride + dense_nbytes
    comp_total_beats = nbeats(3 * IDX_BYTES)
    idx_total_beats = nbeats(n_values * IDX_BYTES) + idx_extra_beats
    if comp_base is None:
        comp_base = align_up(dense_stage_end)
    if idx_base is None:
        idx_base = align_up(comp_base + comp_total_beats * AXI_BYTES)

    metadata_ranges = [
        MemRange("comp_idx", comp_base, comp_base + comp_total_beats * AXI_BYTES),
        MemRange("idx", idx_base, idx_base + idx_total_beats * AXI_BYTES),
    ]
    dense_ranges = [
        MemRange(f"dense_stage[{i}]", dense_stage_base + i * dense_stage_stride, dense_stage_base + i * dense_stage_stride + dense_nbytes)
        for i in range(num_stage_tiles)
    ]
    for dr in dense_ranges:
        for mr in metadata_ranges:
            if dr.overlaps(mr):
                raise ValueError(f"Staging/metadata overlap: {dr} overlaps {mr}. Move comp_base/idx_base or staging.")
    if metadata_ranges[0].overlaps(metadata_ranges[1]):
        raise ValueError(f"Metadata overlap: {metadata_ranges[0]} overlaps {metadata_ranges[1]}")

    metadata = patch_identity_csr_metadata(
        init,
        comp_base=comp_base,
        idx_base=idx_base,
        n_values=n_values,
        idx_extra_beats=idx_extra_beats,
    )
    if patch_gold_staging:
        patch_identity_csr_metadata(
            gold,
            comp_base=comp_base,
            idx_base=idx_base,
            n_values=n_values,
            idx_extra_beats=idx_extra_beats,
        )

    # Capture original tile bytes before poisoning any overlapping output regions.
    original_tiles = {
        base: read_bytes(init, base, dense_nbytes)
        for base in tile_out_bases
    }

    # Put each tile's dense source into a safe staging region.
    gather_rows: list[list[int]] = []
    tile_records: list[dict] = []
    for tile_id, out_base in enumerate(tile_out_bases):
        dense_base = dense_stage_base + tile_id * dense_stage_stride
        tile_data = original_tiles[out_base]
        write_bytes(init, dense_base, tile_data)
        if patch_gold_staging:
            write_bytes(gold, dense_base, tile_data)

        gather_rows.extend(build_one_gather_rows(
            dense_base=dense_base,
            comp_base=comp_base,
            idx_base=idx_base,
            out_base=out_base,
            n_values=n_values,
            n_dense_cols=n_dense_cols,
            idx_extra_beats=idx_extra_beats,
            check_done_read=False,
        ))
        tile_records.append({
            "tile_id": tile_id,
            "dense_base": dense_base,
            "out_base": out_base,
            "n_values": n_values,
            "n_dense_cols": n_dense_cols,
            "bytes_written": gathered_nbytes,
        })

    # Poison after staging all original tile data. Overlapping tiles are OK because
    # all originals have already been captured and each gather writeback restores its range.
    if poison_out:
        for out_base in tile_out_bases:
            write_bytes(init, out_base, [poison_value & 0xFF] * gathered_nbytes)

    if insert_position == "prepend":
        insert_idx = first_controller_access_index(rows)
    elif insert_position == "before-start":
        insert_idx = find_controller_start_index(rows)
    else:
        raise ValueError("insert_position must be 'prepend' or 'before-start'")
    rows = rows[:insert_idx] + gather_rows + rows[insert_idx:]

    write_stimuli(out_dir / "GoldenStimuli.txt", rows)
    write_byte_mem(out_dir / "initial_dram.txt", init)
    write_byte_mem(out_dir / "gold_dram.txt", gold)
    shutil.copy2(tstcfg_path, out_dir / "tstcfg.txt")

    with (out_dir / "gather_tile_plan.csv").open("w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=["tile_id", "dense_base", "out_base", "n_values", "n_dense_cols", "bytes_written"])
        wr.writeheader()
        for rec in tile_records:
            wr.writerow({**rec, "dense_base": f"0x{rec['dense_base']:08X}", "out_base": f"0x{rec['out_base']:08X}"})

    manifest = {
        "mode": "gather_dram_writeback_then_sauria_dma",
        "num_unique_gather_tiles": len(tile_out_bases),
        "n_values": n_values,
        "n_dense_cols": n_dense_cols,
        "dense_stage_base": dense_stage_base,
        "dense_stage_stride": dense_stage_stride,
        "comp_base": comp_base,
        "idx_base": idx_base,
        "idx_extra_beats": idx_extra_beats,
        "metadata": metadata,
        "poison_out": poison_out,
        "poison_value": poison_value & 0xFF,
        "clear_keep_a_bit": clear_keep_a_bit,
        "insert_position": insert_position,
        "input_dir": str(in_dir),
        "output_dir": str(out_dir),
        "tile_out_bases": [f"0x{x:08X}" for x in tile_out_bases],
    }
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2))
    return manifest


# -----------------------------
# Debug-log parsing helpers
# -----------------------------
_DMA_READ_RE = re.compile(r"SAURIA_DMA_EXT_READ\s+(\d+)\s+ar_addr=([0-9a-fA-F]+)\s+ar_len=(\d+)\s+ar_size=(\d+)\s+bytes=(\d+)")


def parse_sauria_dma_reads(log_path: str | Path) -> list[dict]:
    reads: list[dict] = []
    for lineno, line in enumerate(Path(log_path).read_text(errors="replace").splitlines(), start=1):
        m = _DMA_READ_RE.search(line)
        if not m:
            continue
        reads.append({
            "line": lineno,
            "idx": int(m.group(1)),
            "addr": int(m.group(2), 16),
            "ar_len": int(m.group(3)),
            "ar_size": int(m.group(4)),
            "bytes": int(m.group(5)),
            "raw": line,
        })
    return reads


def extract_ifmap_logical_tiles(
    log_path: str | Path,
    *,
    ifmap_region_start: int = 0,
    ifmap_region_end: int = 0x24200,
    tile_bytes: int = 680,
) -> list[dict]:
    """Extract logical IFMAP tiles by combining split AXI reads.

    This matches the current debug trace style: IFMAP reads are external reads in
    a known input region, and a logical tile can be split at a 4 KiB boundary.
    """
    reads = [
        r for r in parse_sauria_dma_reads(log_path)
        if ifmap_region_start <= r["addr"] < ifmap_region_end and r["bytes"] <= tile_bytes
    ]

    tiles: list[dict] = []
    i = 0
    while i < len(reads):
        start = reads[i]["addr"]
        total = reads[i]["bytes"]
        parts = [reads[i]]
        j = i + 1
        while total < tile_bytes and j < len(reads):
            if reads[j]["addr"] != start + total:
                raise ValueError(
                    f"Non-contiguous split tile at read index {i}: start=0x{start:X}, "
                    f"total={total}, next=0x{reads[j]['addr']:X}"
                )
            total += reads[j]["bytes"]
            parts.append(reads[j])
            j += 1
        if total != tile_bytes:
            raise ValueError(f"Could not build full tile at read index {i}; total={total}, tile_bytes={tile_bytes}")
        tiles.append({
            "logical_tile_id": len(tiles),
            "dram_start_addr": start,
            "size_bytes": tile_bytes,
            "size_fp16": tile_bytes // ELEM_BYTES,
            "num_axi_bursts": len(parts),
            "axi_burst_addrs": [p["addr"] for p in parts],
            "axi_burst_sizes": [p["bytes"] for p in parts],
            "axi_read_indices": [p["idx"] for p in parts],
        })
        i = j
    return tiles


def write_ifmap_tiles_csv(path: str | Path, tiles: Sequence[dict]) -> None:
    with Path(path).open("w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow([
            "logical_tile_id", "dram_start_addr_hex", "size_bytes", "size_fp16",
            "num_axi_bursts", "axi_burst_addrs_hex", "axi_burst_sizes_bytes", "axi_read_indices",
        ])
        for t in tiles:
            wr.writerow([
                t["logical_tile_id"],
                f"0x{t['dram_start_addr']:08X}",
                t["size_bytes"],
                t["size_fp16"],
                t["num_axi_bursts"],
                ";".join(f"0x{x:08X}" for x in t["axi_burst_addrs"]),
                ";".join(str(x) for x in t["axi_burst_sizes"]),
                ";".join(str(x) for x in t["axi_read_indices"]),
            ])




def make_gather_restore_check_stimuli(
    *,
    in_dir: str | Path,
    out_dir: str | Path,
    tile_out_base: int = 0,
    n_values: int,
    n_dense_cols: int | None = None,
    dense_stage_base: int = 0x90000,
    dense_stage_stride: int = 0x1000,
    comp_base: int | None = None,
    idx_base: int | None = None,
    idx_extra_beats: int = 1,
    poison_out: bool = True,
    poison_value: int = 0,
) -> dict:
    """Generate a gather-only stimulus that checks whether gather restores one DRAM tile exactly.

    This does NOT run SAURIA. It runs one gather writeback and immediately triggers the
    Verilator gold_dram comparison over [tile_out_base, tile_out_base + n_values*2).

    Use it to isolate whether output mismatches are caused by gather data/layout or by
    later SAURIA execution.
    """
    in_dir = Path(in_dir)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    if n_dense_cols is None:
        n_dense_cols = n_values
    dense_nbytes = n_dense_cols * ELEM_BYTES
    gathered_nbytes = n_values * ELEM_BYTES

    init_path = find_file(in_dir, "initial_dram.txt")
    init = read_byte_mem(init_path)
    gold = read_byte_mem(init_path)  # expected DRAM after gather restore equals original initial DRAM

    def align_up(x: int, a: int = 0x1000) -> int:
        return (x + a - 1) & ~(a - 1)

    comp_total_beats = nbeats(3 * IDX_BYTES)
    idx_total_beats = nbeats(n_values * IDX_BYTES) + idx_extra_beats
    dense_stage_end = dense_stage_base + dense_nbytes
    if comp_base is None:
        comp_base = align_up(dense_stage_end)
    if idx_base is None:
        idx_base = align_up(comp_base + comp_total_beats * AXI_BYTES)

    original_tile = read_bytes(init, tile_out_base, dense_nbytes)
    write_bytes(init, dense_stage_base, original_tile)
    metadata = patch_identity_csr_metadata(init, comp_base=comp_base, idx_base=idx_base, n_values=n_values, idx_extra_beats=idx_extra_beats)

    if poison_out:
        write_bytes(init, tile_out_base, [poison_value & 0xFF] * gathered_nbytes)

    rows = build_one_gather_rows(
        dense_base=dense_stage_base,
        comp_base=comp_base,
        idx_base=idx_base,
        out_base=tile_out_base,
        n_values=n_values,
        n_dense_cols=n_dense_cols,
        idx_extra_beats=idx_extra_beats,
        check_done_read=True,  # trigger final gold_dram comparison immediately after gather
    )

    write_stimuli(out_dir / "GoldenStimuli.txt", rows)
    write_byte_mem(out_dir / "initial_dram.txt", init)
    write_byte_mem(out_dir / "gold_dram.txt", gold)
    # tstcfg format used by current testbench: mode, start, end_exclusive
    (out_dir / "tstcfg.txt").write_text(f"0\n{tile_out_base:X}\n{tile_out_base + gathered_nbytes:X}\n")

    manifest = {
        "mode": "gather_restore_check_only",
        "tile_out_base": f"0x{tile_out_base:08X}",
        "n_values": n_values,
        "n_dense_cols": n_dense_cols,
        "check_start": f"0x{tile_out_base:08X}",
        "check_end_excl": f"0x{tile_out_base + gathered_nbytes:08X}",
        "dense_stage_base": f"0x{dense_stage_base:08X}",
        "comp_base": f"0x{comp_base:08X}",
        "idx_base": f"0x{idx_base:08X}",
        "metadata": metadata,
        "poison_out": poison_out,
    }
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2))
    return manifest
