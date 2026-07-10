#!/usr/bin/env python3
"""
Generate stimuli for DRAM-writeback gather integration:
  gather reads dense/CSR/idx from external DRAM,
  gather writes gathered IFMAP to external DRAM at --gather-out-base,
  SAURIA then starts normally and reloads IFMAP through its standard DMA path.

Important for this SAURIA Verilator testbench:
  The 7th GoldenStimuli field is the FINAL output-check trigger.
  Therefore the REG_DONE read row must be: 0 70000008 0 1 0 1 0
  not:                              ... 1 1
"""
from __future__ import annotations

import argparse, csv, json
from pathlib import Path
from math import ceil, log2

GATHER_BASE = 0x70000000
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

CTRL_BASE       = 0x40000000
CTRL_FLAGS_ADDR = 0x40000064
CTRL_KEEP_A_BIT = 19

AXI_BYTES = 8
ELEM_BYTES = 2     # FP16
IDX_BYTES = 4      # 32-bit indices
NUM_WORDS = 128
N_BLOCKS = 8
WORDS_PER_LINE = AXI_BYTES // ELEM_BYTES
BYTE_SEL_BITS = int(log2(WORDS_PER_LINE))
ADDR_BITS = int(log2(NUM_WORDS))


def parse_int(x: str) -> int:
    return int(x, 0)


def gaddr(off: int) -> int:
    return GATHER_BASE + off


def nbeats(nbytes: int) -> int:
    return ceil(nbytes / AXI_BYTES)


def stim_write(addr: int, data: int) -> list[int]:
    return [data & 0xffffffff, addr & 0xffffffff, 1, 0, 0, 0, 0]


def stim_read(addr: int, expected: int = 0) -> list[int]:
    # Last field MUST stay 0 here. In this testbench, 1 triggers final gold_dram check.
    return [0, addr & 0xffffffff, 0, 1, 0, expected & 0xffffffff, 0]


def stim_wait_gather() -> list[int]:
    return [0, 0, 0, 0, 2, 0, 0]


def pack_dense_index(linear_index: int, block: int = 0) -> int:
    byte_sel = linear_index % WORDS_PER_LINE
    word_addr = linear_index // WORDS_PER_LINE
    if word_addr >= NUM_WORDS:
        raise ValueError(f"linear_index={linear_index} exceeds NUM_WORDS={NUM_WORDS}; split into smaller tiles")
    if block >= N_BLOCKS:
        raise ValueError(f"block={block} exceeds N_BLOCKS={N_BLOCKS}")
    return (block << (ADDR_BITS + BYTE_SEL_BITS)) | (word_addr << BYTE_SEL_BITS) | byte_sel


def read_stimuli(path: Path) -> list[list[int]]:
    rows = []
    for lineno, line in enumerate(path.read_text().splitlines(), 1):
        if not line.strip():
            continue
        vals = [int(x, 16) for x in line.split()]
        if len(vals) != 7:
            raise ValueError(f"Bad GoldenStimuli row at line {lineno}: {line}")
        rows.append(vals)
    return rows


def save_stimuli(path: Path, rows: list[list[int]]) -> None:
    with path.open('w') as f:
        for row in rows:
            f.write(' '.join(f'{x:X}' for x in row) + '\n')


def read_mem(path: Path) -> list[int]:
    return [int(x.strip(), 16) for x in path.read_text().splitlines() if x.strip()]


def save_mem(path: Path, mem: list[int]) -> None:
    with path.open('w') as f:
        for b in mem:
            f.write(f'{b & 0xff:X}\n')


def ensure_len(mem: list[int], n: int) -> None:
    if len(mem) < n:
        mem.extend([0] * (n - len(mem)))


def read_bytes(mem: list[int], addr: int, nbytes: int) -> list[int]:
    ensure_len(mem, addr + nbytes)
    return mem[addr:addr + nbytes]


def write_bytes(mem: list[int], addr: int, data: list[int]) -> None:
    ensure_len(mem, addr + len(data))
    mem[addr:addr + len(data)] = [b & 0xff for b in data]


def zero_range(mem: list[int], addr: int, nbytes: int) -> None:
    write_bytes(mem, addr, [0] * nbytes)


def write_le(mem: list[int], addr: int, value: int, nbytes: int) -> None:
    ensure_len(mem, addr + nbytes)
    for i in range(nbytes):
        mem[addr + i] = (value >> (8 * i)) & 0xff


def u32_le(bs: list[int], off: int) -> int:
    chunk = bs[off:off+4] + [0] * max(0, 4 - len(bs[off:off+4]))
    return sum(chunk[i] << (8*i) for i in range(4))


def copy_file(src: Path, dst: Path) -> None:
    dst.write_text(src.read_text())


def pick(in_dir: Path, name: str) -> Path:
    p = in_dir / name
    if not p.exists():
        raise FileNotFoundError(p)
    return p


def clear_keep_a(stim: list[list[int]]) -> bool:
    for row in stim:
        if row[2] and row[1] == CTRL_FLAGS_ADDR:
            row[0] &= ~(1 << CTRL_KEEP_A_BIT)
            return True
    return False


def first_sauria_cfg_index(stim: list[list[int]]) -> int:
    for i, row in enumerate(stim):
        addr, we, re = row[1], row[2], row[3]
        if (we or re) and CTRL_BASE <= addr < CTRL_BASE + 0x10000:
            return i
    return 0


def check_no_overlap(name_a: str, a0: int, a1: int, name_b: str, b0: int, b1: int) -> None:
    if a0 < b1 and b0 < a1:
        raise ValueError(f"Address range overlap: {name_a} 0x{a0:X}..0x{a1-1:X} vs {name_b} 0x{b0:X}..0x{b1-1:X}")


def build_gather_rows(args, comp_total_beats: int, idx_total_beats: int) -> list[list[int]]:
    return [
        stim_write(gaddr(REG_AR_SIZE), 3),
        stim_write(gaddr(REG_AW_SIZE), 3),
        stim_write(gaddr(REG_AR_ADDR_DENSE_MATRIX), args.dense_base),
        stim_write(gaddr(REG_TOTAL_LEN_DENSE_MATRIX), nbeats(args.n_dense_cols * ELEM_BYTES)),
        stim_write(gaddr(REG_AR_ADDR_COMP_IDX), args.comp_base),
        stim_write(gaddr(REG_TOTAL_LEN_COMP_IDX), comp_total_beats),
        stim_write(gaddr(REG_AR_ADDR_IDX), args.idx_base),
        stim_write(gaddr(REG_TOTAL_LEN_IDX), idx_total_beats),
        stim_write(gaddr(REG_AXI_WR_AW_ADDR_IN), args.gather_out_base),
        stim_write(gaddr(REG_STATUS_REG_N_ROW), args.n_dense_cols - 1),
        stim_write(gaddr(REG_IS_SPMM), 0),
        stim_write(gaddr(REG_START), 1),
        stim_wait_gather(),
        stim_read(gaddr(REG_DONE), 1),   # final field = 0, not benchmark check
    ]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--in-dir', required=True)
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--gather-out-base', '--ifmap-base', dest='gather_out_base', type=parse_int, required=True,
                    help='DRAM address where gather writes IFMAP; must match SAURIA IFMAP DMA source base')
    ap.add_argument('--dense-base', type=parse_int, default=0x90000)
    ap.add_argument('--comp-base', type=parse_int, default=0xA0000)
    ap.add_argument('--idx-base', type=parse_int, default=0xA1000)
    ap.add_argument('--n-values', type=int, default=340)
    ap.add_argument('--n-dense-cols', type=int, default=340)
    ap.add_argument('--idx-extra-beats', type=int, default=1)
    ap.add_argument('--no-poison-out', action='store_true', help='Do not zero the output/IFMAP region before gather')
    ap.add_argument('--poison-value', type=parse_int, default=0x00)
    ap.add_argument('--keep-a', action='store_true', help='Do not clear keep-A. Default clears keep-A for standard SAURIA DMA preload.')
    ap.add_argument('--allow-overlap', action='store_true')
    args = ap.parse_args()

    if args.n_values <= 0 or args.n_dense_cols <= 0:
        raise ValueError('n-values and n-dense-cols must be positive')
    if args.n_values > args.n_dense_cols:
        raise ValueError('n-values cannot exceed n-dense-cols for identity gather')
    if args.idx_extra_beats < 0:
        raise ValueError('idx-extra-beats must be >= 0')

    in_dir = Path(args.in_dir)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    stim = read_stimuli(pick(in_dir, 'GoldenStimuli.txt'))
    init = read_mem(pick(in_dir, 'initial_dram.txt'))
    gold = read_mem(pick(in_dir, 'gold_dram.txt'))

    dense_nbytes = args.n_dense_cols * ELEM_BYTES
    out_nbytes = args.n_values * ELEM_BYTES
    comp_total_beats = nbeats(3 * IDX_BYTES)
    idx_total_beats = nbeats(args.n_values * IDX_BYTES) + args.idx_extra_beats

    ranges = [
        ('dense', args.dense_base, args.dense_base + dense_nbytes),
        ('comp', args.comp_base, args.comp_base + comp_total_beats * AXI_BYTES),
        ('idx', args.idx_base, args.idx_base + idx_total_beats * AXI_BYTES),
        ('out', args.gather_out_base, args.gather_out_base + out_nbytes),
    ]
    if not args.allow_overlap:
        for i, (n0, a0, a1) in enumerate(ranges):
            for n1, b0, b1 in ranges[i+1:]:
                check_no_overlap(n0, a0, a1, n1, b0, b1)

    # Save original IFMAP bytes, stage them as dense source, then optionally poison the original IFMAP/output region.
    original_dense = read_bytes(init, args.gather_out_base, dense_nbytes)
    expected_out = original_dense[:out_nbytes]
    write_bytes(init, args.dense_base, original_dense)
    if not args.no_poison_out:
        write_bytes(init, args.gather_out_base, [args.poison_value & 0xff] * out_nbytes)

    # Identity CSR: one row, indices 0..n_values-1, with full AXI windows zeroed first.
    zero_range(init, args.comp_base, comp_total_beats * AXI_BYTES)
    zero_range(init, args.idx_base, idx_total_beats * AXI_BYTES)
    for i, v in enumerate([0, args.n_values, 0]):
        write_le(init, args.comp_base + i * IDX_BYTES, v, IDX_BYTES)
    for i in range(args.n_values):
        write_le(init, args.idx_base + i * IDX_BYTES, pack_dense_index(i), IDX_BYTES)

    if not args.keep_a and not clear_keep_a(stim):
        raise RuntimeError('Could not find controller flags register 0x40000064 to clear keep-A')

    gather_rows = build_gather_rows(args, comp_total_beats, idx_total_beats)
    insert_idx = first_sauria_cfg_index(stim)
    stim = stim[:insert_idx] + gather_rows + stim[insert_idx:]

    save_stimuli(out_dir / 'GoldenStimuli.txt', stim)
    save_mem(out_dir / 'initial_dram.txt', init)
    save_mem(out_dir / 'gold_dram.txt', gold)
    copy_file(pick(in_dir, 'tstcfg.txt'), out_dir / 'tstcfg.txt')

    with (out_dir / 'expected_gather_output_words32.csv').open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['word_index', 'address', 'expected_u32_hex'])
        for off in range(0, len(expected_out), 4):
            w.writerow([off//4, f'0x{args.gather_out_base + off:08X}', f'0x{u32_le(expected_out, off):08X}'])

    manifest = {
        'mode': 'gather_dram_writeback_then_sauria_dma',
        'critical_note': 'REG_DONE read row has final_check flag 0; final gold_dram check remains only at the original end of GoldenStimuli.',
        'gather_out_base': args.gather_out_base,
        'dense_base': args.dense_base,
        'comp_base': args.comp_base,
        'idx_base': args.idx_base,
        'n_values': args.n_values,
        'n_dense_cols': args.n_dense_cols,
        'REG_TOTAL_LEN_DENSE_MATRIX': nbeats(dense_nbytes),
        'REG_TOTAL_LEN_COMP_IDX': comp_total_beats,
        'REG_TOTAL_LEN_IDX': idx_total_beats,
        'poison_out': not args.no_poison_out,
        'keep_a_cleared': not args.keep_a,
        'first_expected_words32': [f'0x{u32_le(expected_out, off):08X}' for off in range(0, min(len(expected_out), 32), 4)],
    }
    (out_dir / 'manifest.json').write_text(json.dumps(manifest, indent=2))

    print(f'Wrote stimuli to {out_dir}')
    print(f'Gather rows inserted before SAURIA config at index {insert_idx}')
    print(f'IMPORTANT: row after gather wait is: 0 70000008 0 1 0 1 0')
    print(f'Gather writes DRAM out_base: 0x{args.gather_out_base:X}')
    print(f'Dense staging: 0x{args.dense_base:X}')
    print(f'CSR staging:   0x{args.comp_base:X}')
    print(f'IDX staging:   0x{args.idx_base:X}, beats=0x{idx_total_beats:X}')
    print(f'Poison out before gather: {not args.no_poison_out}')
    print('Correct final pass condition: Benchmark passed with no errors / SUCCESS')


if __name__ == '__main__':
    main()
