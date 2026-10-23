"""Telemetry-only CyberGear polling helpers.

Live polling sends CAN request frames, so it is not a passive-bus tool. It must
only be used after the physical motor-power stop path, stand/restraint, CAN
termination, and no-motion power-up checks pass.

This module intentionally avoids any motor enable, calibration, or motion API.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import importlib
import json
from pathlib import Path
import sys
import time
from typing import Any, Iterable

from tocatl_control.cybergear_adapter import (
    CyberGearAdapter,
    DriverBackedCyberGearAdapter,
    READ_ONLY_REQUESTS,
)
from tocatl_diagnostics.motor_map import CAN_BITRATE, MOTORS, Motor


CSV_FIELDS = (
    "timestamp_s",
    "bus",
    "motor_id",
    "leg",
    "joint",
    "fresh",
    "position_rad",
    "velocity_rad_s",
    "torque_estimate_nm",
    "temperature_c",
    "bus_voltage_v",
    "mode",
    "faults",
)

SAFETY_NOTICE = (
    "Telemetry-only mode sends CAN read requests but never enables, zeros, or "
    "commands motion. Run only after the protected harness/no-motion gate passes."
)


@dataclass(frozen=True)
class TelemetrySnapshot:
    timestamp_s: float
    bus: str
    motor_id: int
    leg: int
    joint: str
    fresh: bool
    position_rad: float | None = None
    velocity_rad_s: float | None = None
    torque_estimate_nm: float | None = None
    temperature_c: float | None = None
    bus_voltage_v: float | None = None
    mode: str | None = None
    faults: str | None = None


def snapshot_from_adapter(inventory_motor: Motor, adapter: CyberGearAdapter, timestamp_s: float | None = None) -> TelemetrySnapshot:
    telemetry = adapter.snapshot(timestamp_s=timestamp_s)
    return TelemetrySnapshot(
        timestamp_s=telemetry.timestamp_s,
        bus=inventory_motor.bus,
        motor_id=inventory_motor.motor_id,
        leg=inventory_motor.leg,
        joint=inventory_motor.joint,
        fresh=telemetry.fresh,
        position_rad=telemetry.position_rad,
        velocity_rad_s=telemetry.velocity_rad_s,
        torque_estimate_nm=telemetry.torque_estimate_nm,
        temperature_c=telemetry.temperature_c,
        bus_voltage_v=telemetry.bus_voltage_v,
        mode=telemetry.mode,
        faults=",".join(telemetry.faults) if telemetry.faults else None,
    )


def request_read_only_updates(adapters: Iterable[CyberGearAdapter]) -> None:
    for adapter in adapters:
        adapter.request_read_only_update()


def write_snapshots_csv(path: Path, snapshots: Iterable[TelemetrySnapshot]) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for snapshot in snapshots:
            writer.writerow(asdict(snapshot))


def dry_run_payload(motors: Iterable[Motor] = MOTORS) -> dict[str, Any]:
    return {
        "safety_notice": SAFETY_NOTICE,
        "can_bitrate": CAN_BITRATE,
        "csv_fields": list(CSV_FIELDS),
        "read_only_requests": list(READ_ONLY_REQUESTS),
        "motors": [
            {
                "bus": motor.bus,
                "motor_id": motor.motor_id,
                "leg": motor.leg,
                "joint": motor.joint,
            }
            for motor in motors
        ],
    }


def _import_required(name: str) -> Any:
    try:
        return importlib.import_module(name)
    except ImportError as error:
        raise SystemExit(
            f"Missing optional dependency {name!r}. Install the CyberGear driver "
            "and python-can in the Pi virtual environment before live telemetry."
        ) from error


def _connect_live_adapters(interface: str, channel: str, bitrate: int, motor_ids: list[int]) -> tuple[Any, Any, list[CyberGearAdapter]]:
    can = _import_required("can")
    driver = _import_required("CyberGearDriver")
    bus = can.interface.Bus(interface=interface, channel=channel, bitrate=bitrate)

    def send_message(message: Any) -> None:
        bus.send(
            can.Message(
                arbitration_id=message.arbitration_id,
                data=message.data,
                is_extended_id=message.is_extended_id,
            )
        )

    driver_motors = [driver.CyberGearMotor(motor_id=motor_id, send_message=send_message) for motor_id in motor_ids]
    notifier = can.Notifier(bus, [motor.message_received for motor in driver_motors])
    return bus, notifier, [DriverBackedCyberGearAdapter(motor) for motor in driver_motors]


def live_once(interface: str, channel: str, bus_name: str, delay_s: float, bitrate: int = CAN_BITRATE) -> list[TelemetrySnapshot]:
    inventory = [motor for motor in MOTORS if motor.bus == bus_name]
    bus = notifier = None
    try:
        bus, notifier, adapters = _connect_live_adapters(
            interface=interface,
            channel=channel,
            bitrate=bitrate,
            motor_ids=[motor.motor_id for motor in inventory],
        )
        request_read_only_updates(adapters)
        time.sleep(delay_s)
        now = time.time()
        return [
            snapshot_from_adapter(inventory_motor, adapter, timestamp_s=now)
            for inventory_motor, adapter in zip(inventory, adapters)
        ]
    finally:
        if notifier is not None:
            notifier.stop()
        if bus is not None:
            bus.shutdown()


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="TOCATL CyberGear telemetry-only helper")
    parser.add_argument("--live", action="store_true", help="Send read-only CAN requests")
    parser.add_argument("--interface", default="socketcan")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bus", choices=("can0", "can1"), default="can0")
    parser.add_argument("--bitrate", type=int, default=CAN_BITRATE)
    parser.add_argument("--delay-s", type=float, default=0.2)
    parser.add_argument("--csv", type=Path, help="Write telemetry snapshots to CSV")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if not args.live:
        print(json.dumps(dry_run_payload(), indent=2))
        return

    snapshots = live_once(
        interface=args.interface,
        channel=args.channel,
        bus_name=args.bus,
        delay_s=args.delay_s,
        bitrate=args.bitrate,
    )
    if args.csv:
        write_snapshots_csv(args.csv, snapshots)
    print(json.dumps([asdict(snapshot) for snapshot in snapshots], indent=2))


if __name__ == "__main__":
    main()
