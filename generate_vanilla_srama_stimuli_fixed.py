#!/usr/bin/env python3
"""
Generate the vanilla single-tile SAURIA stimuli required by
sauria_srama_overwrite_test.py.

Expected output:
  test/stimuli_vanilla_for_gather_sram/
    GoldenStimuli.txt
    initial_dram.txt
    gold_dram.txt
    tstcfg.txt

Run from anywhere inside the SAURIA repository, for example:

  python generate_vanilla_srama_stimuli.py

By default the script compiles FP16_8x16_AXI64 before generating the test.
Use --skip-compile only when Test-Sim is already up to date.
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path


REQUIRED_STIMULI = (
    "GoldenStimuli.txt",
    "initial_dram.txt",
    "gold_dram.txt",
    "tstcfg.txt",
)


def find_repo_root(start: Path) -> Path:
    start = start.resolve()
    for candidate in (start, *start.parents):
        if (
            (candidate / "RTL").is_dir()
            and (candidate / "Python").is_dir()
            and (candidate / "test").is_dir()
        ):
            return candidate
    raise FileNotFoundError(
        "SAURIA repository root not found. "
        "Run this script from inside the cloned repository."
    )


def load_environment(repo_root: Path) -> None:
    try:
        import dotenv
    except ImportError as exc:
        raise RuntimeError(
            "python-dotenv is missing. Activate sauria-env and run "
            "`python -m pip install python-dotenv`."
        ) from exc

    for env_file in (
        repo_root / "Python" / "env",
        repo_root / "env",
    ):
        if env_file.exists():
            dotenv.load_dotenv(env_file, override=True)


def compile_sauria(repo_root: Path, version: str) -> None:
    verilator_dir = repo_root / "test" / "verilator"
    compile_script = verilator_dir / "compile_sauria.sh"
    if not compile_script.exists():
        raise FileNotFoundError(compile_script)

    print(f"Compiling {version}...")
    result = subprocess.run(
        ["sh", "./compile_sauria.sh", version],
        cwd=verilator_dir,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )

    log_path = verilator_dir / "verilator_compile_vanilla_srama.log"
    log_path.write_text(result.stdout)

    tail = result.stdout.splitlines()[-80:]
    if tail:
        print("\n".join(tail))

    if result.returncode != 0:
        raise RuntimeError(
            f"Verilator compilation failed with code {result.returncode}. "
            f"Inspect {log_path}."
        )

    print(f"Compilation completed. Log: {log_path}")


def find_generated_file(stimuli_dir: Path, filename: str) -> Path:
    exact = stimuli_dir / filename
    if exact.exists():
        return exact

    matches = sorted(stimuli_dir.glob(filename.replace(".txt", "*.txt")))
    if matches:
        return matches[0]

    raise FileNotFoundError(
        f"{filename} was not generated in {stimuli_dir}"
    )


def copy_stimuli_set(src: Path, dst: Path) -> None:
    dst.mkdir(parents=True, exist_ok=True)
    for filename in REQUIRED_STIMULI:
        source = find_generated_file(src, filename)
        shutil.copy2(source, dst / filename)


def generate(repo_root: Path, version: str, output_dir: Path) -> int:
    try:
        import numpy as np
        import torch
    except ImportError as exc:
        raise RuntimeError(
            "NumPy or PyTorch is missing. Activate the SAURIA environment "
            "created by setup.sh before running this script."
        ) from exc

    python_dir = repo_root / "Python"
    sys.path.insert(0, str(python_dir))

    import src.hw_versions as hwv
    import src.sauria_lib as slib

    random_seed = 7
    torch.manual_seed(random_seed)
    np.random.seed(random_seed)

    # Minimal legal one-tile workload for FP16_8x16_AXI64:
    # A = 16 x 1 x 8, B = 16 x 16 x 1 x 1, C = 16 x 1 x 8.
    c_in = 16
    c_out = 16
    kh = kw = 1
    stride = 1
    dilation = 1

    cw = 8
    ch = 1
    aw = (1 + stride * (cw - 1)) + (1 + dilation * (kw - 1)) - 1
    ah = (1 + stride * (ch - 1)) + (1 + dilation * (kh - 1)) - 1

    with torch.no_grad():
        conv = torch.nn.Conv2d(
            c_in,
            c_out,
            (kh, kw),
            stride=stride,
            dilation=dilation,
            bias=False,
        )
        tensor_a_torch = torch.randn(c_in, ah, aw, dtype=torch.float32)
        tensor_c_torch = conv(tensor_a_torch)

    tensor_a = tensor_a_torch.detach().cpu().numpy().copy()
    tensor_b = conv.weight.detach().cpu().numpy().copy()
    tensor_c = tensor_c_torch.detach().cpu().numpy().copy()
    preload_c = np.zeros_like(tensor_c, dtype=np.float32)

    hw_params = hwv.get_params(version)

    tiling_dict = {
        "C_tile_shape": [16, 1, 8],
        "tile_cin": 16,
        "X_used": 16,
        "Y_used": 8,
    }

    conv_dict = slib.get_conv_dict(
        [tensor_a.shape, tensor_b.shape, tensor_c.shape],
        tiling_dict,
        hw_params,
        d=dilation,
        s=stride,
        preloads=False,
    )

    n_values = (
        conv_dict["A_w_til"]
        * conv_dict["A_h_til"]
        * conv_dict["c_til"]
    )

    if conv_dict["N_total_tiles"] != 1:
        raise AssertionError(
            f"Expected one external tile, got {conv_dict['N_total_tiles']}"
        )
    if n_values != 128:
        raise AssertionError(
            f"Expected 128 gathered FP16 values, got {n_values}"
        )

    test_dir = repo_root / "test"
    stimuli_dir = test_dir / "stimuli"

    print("Generating and checking vanilla SAURIA stimuli...")
    print("A shape:", tensor_a.shape)
    print("B shape:", tensor_b.shape)
    print("C shape:", tensor_c.shape)
    print("N_total_tiles:", conv_dict["N_total_tiles"])
    print("N_VALUES:", n_values)

    # sauria_lib.py currently resolves "../../test/verilator" relative to
    # the process working directory. Reproduce the directory expected by
    # the original SAURIA notebooks, then restore the caller's directory.
    notebook_dir = repo_root / "Python" / "notebooks"
    if not notebook_dir.is_dir():
        raise FileNotFoundError(
            f"Expected SAURIA notebook directory not found: {notebook_dir}"
        )

    caller_cwd = Path.cwd()
    try:
        os.chdir(notebook_dir)
        slib.Conv2d_SAURIA(
            tensor_a,
            tensor_b,
            preload_c,
            tensor_c,
            conv_dict,
            hw_params,
            generate_vcd=False,
            assert_no_errors=True,
            print_statistics=True,
            test_dir=str(test_dir),
            silent=False,
        )
    finally:
        os.chdir(caller_cwd)

    copy_stimuli_set(stimuli_dir, output_dir)

    missing = [
        name for name in REQUIRED_STIMULI
        if not (output_dir / name).exists()
    ]
    if missing:
        raise RuntimeError(
            "Stimuli copy incomplete: " + ", ".join(missing)
        )

    print()
    print("Vanilla stimuli generated successfully:")
    print(output_dir)
    for filename in REQUIRED_STIMULI:
        path = output_dir / filename
        print(f"  {filename}: {path.stat().st_size} bytes")

    return n_values


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--repo-root",
        type=Path,
        help="SAURIA repository root; auto-detected by default.",
    )
    parser.add_argument(
        "--version",
        default="FP16_8x16_AXI64",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        help=(
            "Vanilla output directory. Default: "
            "test/stimuli_vanilla_for_gather_sram"
        ),
    )
    parser.add_argument(
        "--skip-compile",
        action="store_true",
        help="Do not rebuild Test-Sim before generating the workload.",
    )
    args = parser.parse_args()

    repo_root = (
        args.repo_root.resolve()
        if args.repo_root is not None
        else find_repo_root(Path.cwd())
    )

    load_environment(repo_root)

    output_dir = (
        args.out_dir.resolve()
        if args.out_dir is not None
        else repo_root / "test" / "stimuli_vanilla_for_gather_sram"
    )

    if not args.skip_compile:
        compile_sauria(repo_root, args.version)

    n_values = generate(
        repo_root=repo_root,
        version=args.version,
        output_dir=output_dir,
    )

    print()
    print("Next command:")
    print(
        f"python {repo_root / 'sauria_srama_overwrite_test.py'} "
        f"--in-dir {output_dir} "
        f"--out-dir {repo_root / 'test' / 'stimuli'} "
        f"--n-values {n_values}"
    )


if __name__ == "__main__":
    main()
