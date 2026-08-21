#!/usr/bin/env python3
"""Build the optional dynamic-batch armor TensorRT engine.

The generated profile is min=1, opt=8, max=16 at 320x320.  The current
``export.py`` derives opt as half of the requested maximum batch size.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", type=Path, default=PROJECT_ROOT / "models/armor.pt")
    parser.add_argument("--output", type=Path, default=PROJECT_ROOT / "models/armor_batch.engine")
    parser.add_argument("--device", default="0")
    parser.add_argument("--workspace", type=int, default=4)
    args = parser.parse_args()

    weights = args.weights.resolve()
    output = args.output.resolve()
    if not weights.is_file():
        parser.error(f"weights not found: {weights}")
    output.parent.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="armor-batch-") as directory:
        temporary_weights = Path(directory) / "armor_batch.pt"
        shutil.copy2(weights, temporary_weights)
        command = [
            sys.executable,
            str(PROJECT_ROOT / "src" / "export.py"),
            "--weights", str(temporary_weights),
            "--data", str(PROJECT_ROOT / "config" / "armor.yaml"),
            "--include", "engine",
            "--imgsz", "320", "320",
            "--batch-size", "16",
            "--dynamic",
            "--device", str(args.device),
            "--workspace", str(args.workspace),
        ]
        # export.py 依赖根目录下的 models/ 与 utils/，通过 PYTHONPATH 提供给子进程
        env = dict(os.environ, PYTHONPATH=str(PROJECT_ROOT))
        subprocess.run(command, cwd=PROJECT_ROOT, env=env, check=True)
        generated = temporary_weights.with_suffix(".engine")
        if not generated.is_file():
            raise RuntimeError("TensorRT export completed without producing an engine")
        shutil.move(str(generated), str(output))

    print(f"dynamic armor engine written to {output}")
    print("TensorRT profile: min=1, opt=8, max=16, input=3x320x320")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
