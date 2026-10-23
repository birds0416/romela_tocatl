"""Reference geometry recovered from the deployed TOCATL walking controller.

This module is intentionally pure math and metadata. It does not import CAN,
open sockets, enable motors, write zero positions, or command motion.

The values here came from the recovered ``WalkingExcursion_v1.py`` controller.
Treat them as a historical working reference for calibration/gait design, not
as validated final safety limits.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from math import atan2, cos, pi, sin, sqrt


@dataclass(frozen=True)
class Point3:
    """3D point in the leg-local coordinate frame, in millimeters."""

    x_mm: float
    y_mm: float
    z_mm: float


@dataclass(frozen=True)
class LegGeometry:
    """Single-leg geometry constants from the legacy controller."""

    hip_vertical_offset_mm: float = 114.25
    hip_horizontal_offset_mm: float = 45.5561
    upper_link_mm: float = 330.3455
    lower_link_mm: float = 458.0


@dataclass(frozen=True)
class LegacyMotorMapping:
    """Legacy theta-to-motor offsets and software motor windows."""

    hip_yaw_offset_rad: float = 0.0
    hip_pitch_offset_rad: float = (45.44 - 180.0) * pi / 180.0
    knee_relative_offset_rad: float = -((45.44 - 180.0) * pi / 180.0) + ((45.44 - 11.18) * pi / 180.0)
    hip_yaw_min_rad: float = -1.0
    hip_yaw_max_rad: float = 1.0
    hip_pitch_min_rad: float = 0.0
    hip_pitch_max_rad: float = 4.9
    knee_relative_min_rad: float = -5.0
    knee_relative_max_rad: float = 0.0


@dataclass(frozen=True)
class BodyReference:
    """Legacy body placement reference used by the old gait code."""

    body_diameter_mm: float = 600.0
    first_leg_angle_rad: float = 30.0 * pi / 180.0

    @property
    def body_radius_mm(self) -> float:
        return self.body_diameter_mm / 2.0

    @property
    def leg_mount_angles_rad(self) -> tuple[float, ...]:
        return tuple(self.first_leg_angle_rad + k * pi / 3.0 for k in range(6))


DEFAULT_LEG_GEOMETRY = LegGeometry()
LEGACY_MOTOR_MAPPING = LegacyMotorMapping()
BODY_REFERENCE = BodyReference()


def wrap_pi(angle_rad: float) -> float:
    """Wrap an angle to ``[-pi, pi]``."""

    return atan2(sin(angle_rad), cos(angle_rad))


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def foot_position_from_joint_angles(
    hip_yaw_rad: float,
    hip_pitch_rad: float,
    knee_rad: float,
    geometry: LegGeometry = DEFAULT_LEG_GEOMETRY,
) -> Point3:
    """Legacy FK: return foot position from joint angles.

    The recovered controller modeled the hip yaw as a rotation around z, then
    hip pitch and knee pitch around the leg-local y axis.
    """

    c1 = cos(hip_yaw_rad)
    s1 = sin(hip_yaw_rad)

    upper_x = geometry.upper_link_mm * cos(hip_pitch_rad)
    upper_z = -geometry.upper_link_mm * sin(hip_pitch_rad)
    lower_x = geometry.lower_link_mm * cos(hip_pitch_rad + knee_rad)
    lower_z = -geometry.lower_link_mm * sin(hip_pitch_rad + knee_rad)

    link_x = upper_x + lower_x
    link_y = 0.0
    leg_z = geometry.hip_vertical_offset_mm + upper_z + lower_z

    return Point3(
        x_mm=geometry.hip_horizontal_offset_mm + c1 * link_x - s1 * link_y,
        y_mm=s1 * link_x + c1 * link_y,
        z_mm=leg_z,
    )


def inverse_kinematics_from_foot_position(
    foot: Point3,
    geometry: LegGeometry = DEFAULT_LEG_GEOMETRY,
) -> tuple[float, float, float]:
    """Legacy IK branch used by the recovered controller.

    Returns ``(hip_yaw, hip_pitch, knee)`` in radians. The knee branch is the
    positive ``atan2(sqrt(...), C3)`` branch from the old script.
    """

    x = foot.x_mm - geometry.hip_horizontal_offset_mm
    y = foot.y_mm
    z = foot.z_mm - geometry.hip_vertical_offset_mm

    hip_yaw = atan2(y, x)
    radial = sqrt(x * x + y * y)
    reach = sqrt(radial * radial + z * z)

    upper = geometry.upper_link_mm
    lower = geometry.lower_link_mm
    c3 = (reach * reach - upper * upper - lower * lower) / (2.0 * upper * lower)
    c3 = _clamp(c3, -1.0, 1.0)

    knee = atan2(sqrt(max(0.0, 1.0 - c3 * c3)), c3)
    phi = atan2(-z, radial)
    gamma = atan2(lower * sin(knee), upper + lower * cos(knee))
    hip_pitch = phi - gamma

    return hip_yaw, hip_pitch, knee


def theta_to_legacy_motor(
    hip_yaw_rad: float,
    hip_pitch_rad: float,
    knee_rad: float,
    mapping: LegacyMotorMapping = LEGACY_MOTOR_MAPPING,
) -> tuple[float, float, float]:
    """Convert joint angles to the legacy CyberGear motor coordinates."""

    knee_absolute_offset = mapping.hip_pitch_offset_rad + mapping.knee_relative_offset_rad
    motor_hip_yaw = wrap_pi(hip_yaw_rad - mapping.hip_yaw_offset_rad)
    motor_hip_pitch = hip_pitch_rad - mapping.hip_pitch_offset_rad
    motor_knee = -((hip_pitch_rad + knee_rad) - knee_absolute_offset)
    return motor_hip_yaw, motor_hip_pitch, motor_knee


def legacy_motor_to_theta(
    motor_hip_yaw_rad: float,
    motor_hip_pitch_rad: float,
    motor_knee_rad: float,
    mapping: LegacyMotorMapping = LEGACY_MOTOR_MAPPING,
) -> tuple[float, float, float]:
    """Convert legacy motor coordinates back to joint angles."""

    knee_absolute_offset = mapping.hip_pitch_offset_rad + mapping.knee_relative_offset_rad
    hip_yaw = wrap_pi(motor_hip_yaw_rad + mapping.hip_yaw_offset_rad)
    hip_pitch = motor_hip_pitch_rad + mapping.hip_pitch_offset_rad
    knee_absolute = -motor_knee_rad + knee_absolute_offset
    knee = knee_absolute - hip_pitch
    return hip_yaw, hip_pitch, knee


def apply_legacy_motor_windows(
    hip_yaw_rad: float,
    hip_pitch_rad: float,
    knee_rad: float,
    mapping: LegacyMotorMapping = LEGACY_MOTOR_MAPPING,
) -> tuple[float, float, float]:
    """Apply the old script's motor-coordinate windows, then return theta.

    This is useful for reproducing the old gait code in simulation, but it is
    not a substitute for the new calibration/limit file.
    """

    motor_hip_yaw, motor_hip_pitch, motor_knee = theta_to_legacy_motor(
        hip_yaw_rad,
        hip_pitch_rad,
        knee_rad,
        mapping,
    )
    motor_hip_yaw = _clamp(motor_hip_yaw, mapping.hip_yaw_min_rad, mapping.hip_yaw_max_rad)
    motor_hip_pitch = _clamp(motor_hip_pitch, mapping.hip_pitch_min_rad, mapping.hip_pitch_max_rad)
    relative_knee = _clamp(
        motor_knee - motor_hip_pitch,
        mapping.knee_relative_min_rad,
        mapping.knee_relative_max_rad,
    )
    motor_knee = motor_hip_pitch + relative_knee
    return legacy_motor_to_theta(motor_hip_yaw, motor_hip_pitch, motor_knee, mapping)


def reference_summary() -> dict[str, object]:
    """Return JSON-safe legacy reference metadata."""

    return {
        "safety_notice": (
            "Reference-only geometry extracted from legacy code; this module "
            "does not talk to CAN or command motors."
        ),
        "leg_geometry": asdict(DEFAULT_LEG_GEOMETRY),
        "body": {
            "body_diameter_mm": BODY_REFERENCE.body_diameter_mm,
            "body_radius_mm": BODY_REFERENCE.body_radius_mm,
            "leg_mount_angles_rad": list(BODY_REFERENCE.leg_mount_angles_rad),
        },
        "legacy_motor_mapping": asdict(LEGACY_MOTOR_MAPPING),
    }


def main() -> None:
    print(json.dumps(reference_summary(), indent=2))


if __name__ == "__main__":
    main()
