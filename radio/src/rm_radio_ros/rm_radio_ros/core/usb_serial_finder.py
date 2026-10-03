"""Find USB serial device nodes and stable symlink paths."""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

try:
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - exercised only on systems without pyserial
    list_ports = None


TTY_PATTERNS = (
    "/dev/ttyUSB*",
    "/dev/ttyACM*",
    "/dev/ttyCH341USB*",
)


@dataclass(frozen=True)
class UsbSerialPort:
    device: str
    stable_path: str | None = None
    description: str | None = None
    hwid: str | None = None
    vid: str | None = None
    pid: str | None = None
    serial_number: str | None = None
    manufacturer: str | None = None
    product: str | None = None

    @property
    def searchable_text(self) -> str:
        return " ".join(
            value
            for value in (
                self.device,
                self.stable_path,
                self.description,
                self.hwid,
                self.vid,
                self.pid,
                self.serial_number,
                self.manufacturer,
                self.product,
            )
            if value
        ).lower()


def _realpath(path: str) -> str:
    return os.path.realpath(path)


def stable_serial_links(by_id_dir: str = "/dev/serial/by-id") -> dict[str, str]:
    links: dict[str, str] = {}
    for link in sorted(Path(by_id_dir).glob("*")):
        if not link.is_symlink():
            continue
        links[_realpath(str(link))] = str(link)
    return links


def _format_usb_id(value: int | None) -> str | None:
    if value is None:
        return None
    return f"{value:04x}"


def _ports_from_pyserial(stable_links: dict[str, str]) -> list[UsbSerialPort]:
    if list_ports is None:
        return []

    ports: list[UsbSerialPort] = []
    for port in sorted(list_ports.comports(include_links=False), key=lambda p: p.device):
        device = port.device
        if not any(device.startswith(prefix.rstrip("*")) for prefix in TTY_PATTERNS):
            continue
        ports.append(
            UsbSerialPort(
                device=device,
                stable_path=stable_links.get(_realpath(device)),
                description=port.description,
                hwid=port.hwid,
                vid=_format_usb_id(port.vid),
                pid=_format_usb_id(port.pid),
                serial_number=port.serial_number,
                manufacturer=port.manufacturer,
                product=port.product,
            )
        )
    return ports


def _ports_from_glob(stable_links: dict[str, str]) -> list[UsbSerialPort]:
    devices = sorted({device for pattern in TTY_PATTERNS for device in glob.glob(pattern)})
    return [
        UsbSerialPort(device=device, stable_path=stable_links.get(_realpath(device)))
        for device in devices
    ]


def find_usb_serial_ports() -> list[UsbSerialPort]:
    stable_links = stable_serial_links()
    ports = _ports_from_pyserial(stable_links)
    if ports:
        return ports
    return _ports_from_glob(stable_links)


def filter_ports(ports: Iterable[UsbSerialPort], text: str | None) -> list[UsbSerialPort]:
    if not text:
        return list(ports)
    needle = text.lower()
    return [port for port in ports if needle in port.searchable_text]


def _port_key(port: UsbSerialPort) -> str:
    return port.stable_path or port.device


def diff_ports(before: Iterable[UsbSerialPort], after: Iterable[UsbSerialPort]) -> list[UsbSerialPort]:
    before_keys = {_port_key(port) for port in before}
    before_devices = {port.device for port in before}
    return [
        port
        for port in after
        if _port_key(port) not in before_keys and port.device not in before_devices
    ]


def _print_table(ports: list[UsbSerialPort]) -> None:
    if not ports:
        print("No USB serial ports found.")
        print("Plug in the USB serial adapter, then run this command again.")
        return

    print("USB serial ports:")
    for index, port in enumerate(ports, start=1):
        print(f"{index}. current: {port.device}")
        if port.stable_path:
            print(f"   stable : {port.stable_path}")
        if port.description:
            print(f"   desc   : {port.description}")
        ids = []
        if port.vid and port.pid:
            ids.append(f"vid:pid={port.vid}:{port.pid}")
        if port.serial_number:
            ids.append(f"serial={port.serial_number}")
        if port.manufacturer:
            ids.append(f"mfg={port.manufacturer}")
        if port.product:
            ids.append(f"product={port.product}")
        if ids:
            print(f"   info   : {', '.join(ids)}")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="List USB serial port numbers such as /dev/ttyUSB0 and stable /dev/serial/by-id paths.",
    )
    parser.add_argument(
        "-f",
        "--filter",
        help="Only show ports whose device, stable path, VID/PID, serial number, manufacturer, or product contains this text.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print machine-readable JSON.",
    )
    parser.add_argument(
        "--first",
        action="store_true",
        help="Print only the first matching current device path, for use in shell scripts.",
    )
    parser.add_argument(
        "--stable",
        action="store_true",
        help="With --first, prefer the stable /dev/serial/by-id path when available.",
    )
    parser.add_argument(
        "--watch",
        action="store_true",
        help="Record current ports, wait for a USB serial adapter to be plugged in, then show only new ports.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="Seconds to wait with --watch. Default: 30.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    if args.watch:
        before = filter_ports(find_usb_serial_ports(), args.filter)
        print("Current USB serial ports before plug-in:")
        _print_table(before)
        print()
        print(f"Now plug in the referee-system USB serial adapter. Waiting up to {args.timeout:.0f}s...")
        deadline = time.monotonic() + max(args.timeout, 1.0)
        new_ports: list[UsbSerialPort] = []
        while time.monotonic() < deadline:
            time.sleep(0.5)
            after = filter_ports(find_usb_serial_ports(), args.filter)
            new_ports = diff_ports(before, after)
            if new_ports:
                break
        if not new_ports:
            print("No new USB serial port was detected.")
            print("Check cable, power, USB permissions, or run again while physically re-plugging the adapter.")
            return 1
        print()
        print("New USB serial port(s):")
        _print_table(new_ports)
        recommended = new_ports[0].stable_path or new_ports[0].device
        print()
        print(f"Recommended REFEREE_PORT={recommended}")
        return 0

    ports = filter_ports(find_usb_serial_ports(), args.filter)

    if args.json:
        print(json.dumps([asdict(port) for port in ports], ensure_ascii=False, indent=2))
        return 0 if ports else 1

    if args.first:
        if not ports:
            return 1
        first = ports[0]
        print(first.stable_path if args.stable and first.stable_path else first.device)
        return 0

    _print_table(ports)
    if len(ports) > 1:
        print()
        print("Tip: use --filter TEXT to narrow the result, or use the stable path in REFEREE_PORT.")
    return 0 if ports else 1


if __name__ == "__main__":
    sys.exit(main())
