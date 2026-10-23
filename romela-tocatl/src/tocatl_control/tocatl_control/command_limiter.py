"""Pure command limiting for future TOCATL motor commands.

This module produces bounded command objects or explicit rejection reasons. It
does not import CAN, talk to a motor driver, enable motors, write zero position,
or send motion commands.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite, pi

from tocatl_control.calibration import JointCalibration
from tocatl_control.cybergear_adapter import MotorTelemetry
from tocatl_control.safety_state import SafetyState


TWO_OUTPUT_RPM_RAD_S = 2.0 * 2.0 * pi / 60.0


@dataclass(frozen=True)
class CommandLimits:
    """Software command envelope for early single-motor tests."""

    max_abs_velocity_rad_s: float = TWO_OUTPUT_RPM_RAD_S
    max_abs_torque_nm: float = 0.5
    max_position_step_rad: float = 0.10


@dataclass(frozen=True)
class MotorCommandRequest:
    """Desired joint-frame command before calibration transform."""

    motor_id: int
    position_rad: float
    velocity_rad_s: float = 0.0
    torque_nm: float = 0.0


@dataclass(frozen=True)
class BoundedMotorCommand:
    """Driver-facing command after calibration transform and limit bounding."""

    motor_id: int
    motor_position_rad: float
    motor_velocity_rad_s: float
    torque_nm: float


@dataclass(frozen=True)
class CommandDecision:
    accepted: bool
    command: BoundedMotorCommand | None
    reasons: tuple[str, ...] = ()


def _finite(name: str, value: float | None) -> str | None:
    if value is None or not isfinite(value):
        return f"{name} must be finite"
    return None


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _calibration_errors(calibration: JointCalibration) -> list[str]:
    errors: list[str] = []
    if calibration.sign not in (-1, 1):
        errors.append("calibration sign must be -1 or 1")
    for field in (
        "zero_offset_rad",
        "min_position_rad",
        "max_position_rad",
        "park_position_rad",
    ):
        if (error := _finite(f"calibration {field}", getattr(calibration, field))) is not None:
            errors.append(error)
    if (
        calibration.min_position_rad is not None
        and calibration.max_position_rad is not None
        and calibration.min_position_rad >= calibration.max_position_rad
    ):
        errors.append("calibration min_position_rad must be < max_position_rad")
    return errors


def limit_motor_command(
    request: MotorCommandRequest,
    calibration: JointCalibration,
    telemetry: MotorTelemetry,
    safety_state: SafetyState,
    limits: CommandLimits = CommandLimits(),
) -> CommandDecision:
    reasons: list[str] = []

    if safety_state != SafetyState.ARMED:
        reasons.append(f"safety state is {safety_state.value}, not ARMED")

    if request.motor_id != calibration.motor_id:
        reasons.append(f"request motor_id {request.motor_id} does not match calibration motor_id {calibration.motor_id}")
    if request.motor_id != telemetry.motor_id:
        reasons.append(f"request motor_id {request.motor_id} does not match telemetry motor_id {telemetry.motor_id}")

    for field, value in (
        ("request position_rad", request.position_rad),
        ("request velocity_rad_s", request.velocity_rad_s),
        ("request torque_nm", request.torque_nm),
        ("limit max_abs_velocity_rad_s", limits.max_abs_velocity_rad_s),
        ("limit max_abs_torque_nm", limits.max_abs_torque_nm),
        ("limit max_position_step_rad", limits.max_position_step_rad),
    ):
        if (error := _finite(field, value)) is not None:
            reasons.append(error)

    if limits.max_abs_velocity_rad_s < 0:
        reasons.append("limit max_abs_velocity_rad_s must be nonnegative")
    if limits.max_abs_torque_nm < 0:
        reasons.append("limit max_abs_torque_nm must be nonnegative")
    if limits.max_position_step_rad < 0:
        reasons.append("limit max_position_step_rad must be nonnegative")

    reasons.extend(_calibration_errors(calibration))

    if not telemetry.fresh:
        reasons.append("telemetry is stale or missing")
    if telemetry.faults:
        reasons.append(f"active motor faults: {','.join(telemetry.faults)}")
    if telemetry.position_rad is None or not isfinite(telemetry.position_rad):
        reasons.append("telemetry position_rad must be finite")

    if reasons:
        return CommandDecision(accepted=False, command=None, reasons=tuple(reasons))

    assert calibration.sign in (-1, 1)
    assert calibration.zero_offset_rad is not None
    assert calibration.min_position_rad is not None
    assert calibration.max_position_rad is not None
    assert telemetry.position_rad is not None

    if not calibration.min_position_rad <= request.position_rad <= calibration.max_position_rad:
        return CommandDecision(
            accepted=False,
            command=None,
            reasons=(
                f"requested position {request.position_rad:.4f} rad outside "
                f"[{calibration.min_position_rad:.4f}, {calibration.max_position_rad:.4f}] rad",
            ),
        )

    if abs(request.position_rad - telemetry.position_rad) > limits.max_position_step_rad:
        return CommandDecision(
            accepted=False,
            command=None,
            reasons=(
                f"requested step {abs(request.position_rad - telemetry.position_rad):.4f} rad exceeds "
                f"limit {limits.max_position_step_rad:.4f} rad",
            ),
        )

    bounded_velocity = _clamp(
        request.velocity_rad_s,
        -limits.max_abs_velocity_rad_s,
        limits.max_abs_velocity_rad_s,
    )
    bounded_torque = _clamp(request.torque_nm, -limits.max_abs_torque_nm, limits.max_abs_torque_nm)
    motor_position = calibration.sign * (request.position_rad + calibration.zero_offset_rad)
    motor_velocity = calibration.sign * bounded_velocity

    return CommandDecision(
        accepted=True,
        command=BoundedMotorCommand(
            motor_id=request.motor_id,
            motor_position_rad=motor_position,
            motor_velocity_rad_s=motor_velocity,
            torque_nm=bounded_torque,
        ),
    )
