"""Reference gait metadata recovered from the deployed TOCATL controller.

This module contains no ROS, CAN, joystick, or motor-driver code. It preserves
the useful gait timing/phase constants from ``WalkingExcursion_v1.py`` so new
gait work can start from evidence without reviving the old auto-start script.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from math import pi


@dataclass(frozen=True)
class GaitDefinition:
    name: str
    cycle_time_s: float
    duty_factor: float
    phase_offsets: tuple[float, ...]
    note: str


@dataclass(frozen=True)
class LegacyWalkingEnvelope:
    """Joystick/gait envelope values from the old walking controller."""

    default_stride_length_mm: float = 175.0
    default_step_height_mm: float = 215.0
    stride_length_min_mm: float = 20.0
    stride_length_max_mm: float = 400.0
    step_height_min_mm: float = 0.0
    step_height_max_mm: float = 400.0
    foot_speed_max_mm_s: float = 400.0
    cycle_fast_s: float = 1.2
    cycle_slow_s: float = 4.5
    return_cycle_time_s: float = 1.35
    body_speed_max_mm_s: float = 140.0
    body_yaw_rate_max_rad_s: float = 35.0 * pi / 180.0
    hip_pitch_support_torque_max_nm: float = 9.25
    command_stream_hz: float = 200.0
    estop_damping_kd: float = 8.0
    estop_damping_time_s: float = 1.25


WAVE_STAGGERED = GaitDefinition(
    name="wave",
    cycle_time_s=1.5,
    duty_factor=5.0 / 6.0,
    phase_offsets=(5 / 6, 1 / 3, 2 / 3, 1 / 6, 1 / 2, 0.0),
    note="One leg swings at a time in the recovered controller.",
)

RIPPLE = GaitDefinition(
    name="ripple",
    cycle_time_s=1.5,
    duty_factor=2.0 / 3.0,
    phase_offsets=(0.0, 1 / 3, 2 / 3, 0.0, 1 / 3, 2 / 3),
    note="Recovered intermediate gait; pairs share phase offsets.",
)

TRIPOD = GaitDefinition(
    name="tripod",
    cycle_time_s=1.5,
    duty_factor=0.5,
    phase_offsets=(0.0, 0.5, 0.0, 0.5, 0.0, 0.5),
    note="Alternating tripod gait from the recovered controller.",
)

GAITS = (WAVE_STAGGERED, RIPPLE, TRIPOD)
GAIT_BY_NAME = {gait.name: gait for gait in GAITS}
LEGACY_WALKING_ENVELOPE = LegacyWalkingEnvelope()


def phase_for_leg(gait: GaitDefinition, leg_index: int, global_phase: float) -> tuple[float, bool, float]:
    """Return old-gait phase info for zero-based ``leg_index``.

    Output is ``(u, is_stance, s)``:

    - ``u`` is the leg phase in ``[0, 1)``.
    - ``is_stance`` is true when ``u < duty_factor``.
    - ``s`` is normalized progress within stance or swing.
    """

    if not 0 <= leg_index < len(gait.phase_offsets):
        raise ValueError("leg_index must be 0..5")
    if not 0.0 < gait.duty_factor < 1.0:
        raise ValueError("duty_factor must be between 0 and 1")

    u = (global_phase + gait.phase_offsets[leg_index]) % 1.0
    if u < gait.duty_factor:
        return u, True, u / gait.duty_factor
    return u, False, (u - gait.duty_factor) / (1.0 - gait.duty_factor)


def gait_sequence() -> tuple[str, ...]:
    return tuple(gait.name for gait in GAITS)


def reference_summary() -> dict[str, object]:
    return {
        "safety_notice": (
            "Reference-only gait constants extracted from legacy code; this "
            "module does not talk to CAN or command motors."
        ),
        "gait_sequence": list(gait_sequence()),
        "gaits": [asdict(gait) for gait in GAITS],
        "walking_envelope": asdict(LEGACY_WALKING_ENVELOPE),
    }


def main() -> None:
    print(json.dumps(reference_summary(), indent=2))


if __name__ == "__main__":
    main()
