"""Stable CyberGear adapter boundary for TOCATL control code.

The rest of the robot should not depend directly on a specific CyberGear Python
driver's attribute names. This module defines the small telemetry surface we
need first, plus a read-only wrapper around driver motor objects.
"""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any, Protocol


READ_ONLY_REQUESTS = ("request_motor_state", "request_motor_fault_status")


class ReadOnlyCyberGearDriverMotor(Protocol):
    motor_id: int
    state: dict[str, Any]
    faults: dict[str, bool] | list[str]

    def request_motor_state(self) -> None:
        ...

    def request_motor_fault_status(self) -> None:
        ...


@dataclass(frozen=True)
class MotorTelemetry:
    timestamp_s: float
    motor_id: int
    fresh: bool
    position_rad: float | None = None
    velocity_rad_s: float | None = None
    torque_estimate_nm: float | None = None
    temperature_c: float | None = None
    bus_voltage_v: float | None = None
    mode: str | None = None
    faults: tuple[str, ...] = ()


class CyberGearAdapter(Protocol):
    motor_id: int

    def request_read_only_update(self) -> None:
        """Ask the motor for telemetry/fault state without enabling or motion."""

    def snapshot(self, timestamp_s: float | None = None) -> MotorTelemetry:
        """Return the latest locally cached telemetry."""


def get_first(mapping: dict[str, Any], names: tuple[str, ...]) -> Any:
    for name in names:
        if name in mapping:
            return mapping[name]
    return None


def active_faults(raw_faults: Any) -> tuple[str, ...]:
    if isinstance(raw_faults, dict):
        return tuple(sorted(str(name) for name, active in raw_faults.items() if active))
    if isinstance(raw_faults, (list, tuple, set)):
        return tuple(sorted(str(name) for name in raw_faults))
    return (str(raw_faults),) if raw_faults else ()


class DriverBackedCyberGearAdapter:
    """Read-only wrapper for a CyberGear driver motor object."""

    def __init__(self, driver_motor: ReadOnlyCyberGearDriverMotor):
        self.driver_motor = driver_motor
        self.motor_id = driver_motor.motor_id

    def request_read_only_update(self) -> None:
        self.driver_motor.request_motor_state()
        self.driver_motor.request_motor_fault_status()

    def snapshot(self, timestamp_s: float | None = None) -> MotorTelemetry:
        state = getattr(self.driver_motor, "state", {}) or {}
        return MotorTelemetry(
            timestamp_s=time.time() if timestamp_s is None else timestamp_s,
            motor_id=self.motor_id,
            fresh=bool(state),
            position_rad=get_first(state, ("position", "position_rad", "pos")),
            velocity_rad_s=get_first(state, ("velocity", "velocity_rad_s", "vel")),
            torque_estimate_nm=get_first(state, ("torque", "torque_estimate_nm", "tau")),
            temperature_c=get_first(state, ("temperature", "temperature_c", "temp")),
            bus_voltage_v=get_first(state, ("bus_voltage", "bus_voltage_v", "voltage")),
            mode=get_first(state, ("mode", "run_mode")),
            faults=active_faults(getattr(self.driver_motor, "faults", None)),
        )
