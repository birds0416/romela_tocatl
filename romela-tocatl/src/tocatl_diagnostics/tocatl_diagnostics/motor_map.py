"""Canonical motor/CAN inventory inferred from the deployed TOCATL controller.

This module deliberately has no CAN or motor-driver dependency. Importing it and
running its CLI cannot transmit, enable, zero, or command a motor.
"""

from dataclasses import asdict, dataclass
import json
from typing import Iterable


CAN_BITRATE = 1_000_000


@dataclass(frozen=True)
class Motor:
    motor_id: int
    leg: int
    joint: str
    bus: str


def _build_motor_map() -> tuple[Motor, ...]:
    buses = {1: "can1", 2: "can0", 3: "can0", 4: "can0", 5: "can1", 6: "can1"}
    joints = {1: "hip_yaw", 2: "hip_pitch", 3: "knee"}
    return tuple(
        Motor(motor_id=leg * 10 + joint_number, leg=leg, joint=joint, bus=buses[leg])
        for leg in range(1, 7)
        for joint_number, joint in joints.items()
    )


MOTORS = _build_motor_map()


def as_records(motors: Iterable[Motor] = MOTORS) -> list[dict[str, object]]:
    return [asdict(motor) for motor in motors]


def main() -> None:
    """Print the static inventory as JSON without accessing robot hardware."""
    print(json.dumps({"can_bitrate": CAN_BITRATE, "motors": as_records()}, indent=2))


if __name__ == "__main__":
    main()

