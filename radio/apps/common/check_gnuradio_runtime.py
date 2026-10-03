#!/usr/bin/env python3
"""Fail closed on the known-bad GNU Radio 3.10.1 IIO runtime.

The machine can contain more than one GNU Radio ABI at once.  Checking only
``gnuradio-config-info`` is therefore insufficient: this script imports the
same Python module used by the radio nodes and inspects the shared libraries
resolved for its gr-iio extension.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import re
import subprocess
import sys
from pathlib import Path


MIN_GNURADIO = (3, 10, 7, 0)
LATEST_STABLE_GNURADIO = (3, 10, 12, 0)
LATEST_STABLE_LIBIIO = (0, 26)
REJECTED_GNURADIO_PREFIXES = ((3, 10, 1),)


def _version_tuple(value: str, width: int = 4) -> tuple[int, ...]:
    numbers = [int(item) for item in re.findall(r"\d+", str(value))[:width]]
    return tuple((numbers + [0] * width)[:width])


def _libiio_version() -> tuple[int, int, str]:
    library = ctypes.CDLL("libiio.so.0")
    major = ctypes.c_uint()
    minor = ctypes.c_uint()
    tag = ctypes.create_string_buffer(32)
    library.iio_library_get_version(
        ctypes.byref(major), ctypes.byref(minor), tag
    )
    return major.value, minor.value, tag.value.decode(errors="replace")


def audit_runtime() -> dict:
    from gnuradio import gr
    from gnuradio.iio import iio_python

    extension = Path(iio_python.__file__).resolve()
    linked = subprocess.run(
        ["ldd", str(extension)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    libraries = {}
    for name in ("libgnuradio-iio", "libgnuradio-runtime", "libiio"):
        match = re.search(rf"^\s*{re.escape(name)}[^ ]*\s+=>\s+(\S+)", linked, re.M)
        libraries[name] = str(Path(match.group(1)).resolve()) if match else ""

    libiio_major, libiio_minor, libiio_tag = _libiio_version()
    return {
        "python": sys.executable,
        "gnuradio_version": str(gr.version()),
        "gnuradio_iio_extension": str(extension),
        "linked_libraries": libraries,
        "libiio_version": f"{libiio_major}.{libiio_minor}",
        "libiio_tag": libiio_tag,
    }


def validate(report: dict, *, require_latest: bool = False) -> list[str]:
    errors = []
    version = _version_tuple(report["gnuradio_version"])
    linked_text = " ".join(report["linked_libraries"].values())
    if version < MIN_GNURADIO:
        errors.append(
            f"GNU Radio {report['gnuradio_version']} is below the supported "
            f"minimum {'.'.join(map(str, MIN_GNURADIO))}"
        )
    if any(version[:3] == prefix for prefix in REJECTED_GNURADIO_PREFIXES):
        errors.append("GNU Radio 3.10.1 is rejected because its TX amplitude is not trusted")
    if ".3.10.1" in linked_text:
        errors.append("the imported gr-iio extension resolved a GNU Radio 3.10.1 shared library")

    expected_abi = ".".join(map(str, version[:3]))
    for name in ("libgnuradio-iio", "libgnuradio-runtime"):
        path = report["linked_libraries"].get(name, "")
        if not path:
            errors.append(f"{name} was not resolved by ldd")
        elif expected_abi not in Path(path).name:
            errors.append(
                f"{name} ABI mismatch: Python reports {report['gnuradio_version']}, "
                f"but ldd resolved {path}"
            )
    if not report["linked_libraries"].get("libiio"):
        errors.append("libiio was not resolved by ldd")

    if require_latest:
        if version != LATEST_STABLE_GNURADIO:
            errors.append(
                f"GNU Radio is not the audited latest stable release "
                f"{'.'.join(map(str, LATEST_STABLE_GNURADIO))}"
            )
        libiio = _version_tuple(report["libiio_version"], width=2)
        if libiio != LATEST_STABLE_LIBIIO:
            errors.append(
                f"libiio is not the audited latest stable release "
                f"{'.'.join(map(str, LATEST_STABLE_LIBIIO))}"
            )
    return errors


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--require-latest", action="store_true")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()
    try:
        report = audit_runtime()
        errors = validate(report, require_latest=args.require_latest)
    except Exception as exc:  # noqa: BLE001 - startup audit must fail closed
        print(f"[runtime-audit][ERROR] {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    if not args.quiet:
        print("[runtime-audit] " + json.dumps(report, ensure_ascii=False, sort_keys=True))
    for error in errors:
        print(f"[runtime-audit][ERROR] {error}", file=sys.stderr)
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
