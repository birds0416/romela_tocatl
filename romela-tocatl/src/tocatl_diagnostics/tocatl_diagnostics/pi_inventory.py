"""Collect read-only Raspberry Pi, USB, network-link, and service metadata.

This module does not import a CAN library, open a CAN socket, configure a link,
or transmit a frame. It is intended for use while the robot motor rail is off.
"""

import json
from pathlib import Path
import platform
import subprocess
from typing import Callable


COMMANDS = (
    ("lsusb",),
    ("ip", "-details", "link", "show"),
    ("systemctl", "--user", "is-enabled", "tocatl.service"),
    ("systemctl", "--user", "is-active", "tocatl.service"),
)

USB_FIELDS = ("idVendor", "idProduct", "manufacturer", "product", "serial")


def _read_text(path: Path) -> str | None:
    try:
        return path.read_text(errors="replace").strip("\x00\n ")
    except (OSError, UnicodeError):
        return None


def usb_devices(root: Path = Path("/sys/bus/usb/devices")) -> list[dict[str, str]]:
    devices = []
    if not root.exists():
        return devices
    for device in sorted(root.iterdir()):
        record = {field: value for field in USB_FIELDS if (value := _read_text(device / field))}
        if "idVendor" in record and "idProduct" in record:
            record["sysfs_name"] = device.name
            devices.append(record)
    return devices


def run_command(command: tuple[str, ...]) -> dict[str, object]:
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=10, check=False)
        return {
            "command": list(command),
            "returncode": result.returncode,
            "stdout": result.stdout.strip(),
            "stderr": result.stderr.strip(),
        }
    except (FileNotFoundError, subprocess.TimeoutExpired) as error:
        return {"command": list(command), "error": str(error)}


def collect(runner: Callable[[tuple[str, ...]], dict[str, object]] = run_command) -> dict[str, object]:
    model = _read_text(Path("/proc/device-tree/model"))
    return {
        "safety_notice": "Read-only inventory; run with robot motor rail off.",
        "platform": {"model": model, "hostname": platform.node(), "kernel": platform.release()},
        "usb_devices": usb_devices(),
        "commands": [runner(command) for command in COMMANDS],
    }


def main() -> None:
    print(json.dumps(collect(), indent=2))


if __name__ == "__main__":
    main()
