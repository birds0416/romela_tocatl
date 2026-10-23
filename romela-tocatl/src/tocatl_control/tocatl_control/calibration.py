"""Versioned calibration document support for TOCATL.

Calibration is software metadata, not a motor command. This module does not
talk to CAN, enable motors, or write CyberGear zero positions. Its job is to
make the arming precondition "valid calibration" concrete and testable.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
import json
from pathlib import Path
import sys
from typing import Any


SCHEMA_VERSION = 1
JOINTS = ("hip_yaw", "hip_pitch", "knee")
CAN_BY_LEG = {1: "can1", 2: "can0", 3: "can0", 4: "can0", 5: "can1", 6: "can1"}
JOINT_SUFFIX = {"hip_yaw": 1, "hip_pitch": 2, "knee": 3}


@dataclass(frozen=True)
class JointCalibration:
    motor_id: int
    leg: int
    joint: str
    bus: str
    sign: int | None
    zero_offset_rad: float | None
    min_position_rad: float | None
    max_position_rad: float | None
    park_position_rad: float | None
    method: str
    notes: str = ""


@dataclass(frozen=True)
class CalibrationDocument:
    schema_version: int
    robot: str
    robot_revision: str
    created_utc: str
    source: str
    joints: list[JointCalibration]


def expected_motor_id(leg: int, joint: str) -> int:
    return leg * 10 + JOINT_SUFFIX[joint]


def expected_joints() -> list[tuple[int, int, str, str]]:
    return [
        (expected_motor_id(leg, joint), leg, joint, CAN_BY_LEG[leg])
        for leg in range(1, 7)
        for joint in JOINTS
    ]


def calibration_template() -> CalibrationDocument:
    """Return a complete-shaped but intentionally incomplete calibration file."""

    return CalibrationDocument(
        schema_version=SCHEMA_VERSION,
        robot="tocatl",
        robot_revision="spider-art-current",
        created_utc=datetime.now(UTC).replace(microsecond=0).isoformat(),
        source="template; fill values from supervised calibration procedure",
        joints=[
            JointCalibration(
                motor_id=motor_id,
                leg=leg,
                joint=joint,
                bus=bus,
                sign=None,
                zero_offset_rad=None,
                min_position_rad=None,
                max_position_rad=None,
                park_position_rad=None,
                method="TODO: mechanical reference / telemetry procedure",
                notes="Placeholder values are invalid for arming.",
            )
            for motor_id, leg, joint, bus in expected_joints()
        ],
    )


def document_to_dict(document: CalibrationDocument) -> dict[str, Any]:
    return asdict(document)


def document_from_dict(data: dict[str, Any]) -> CalibrationDocument:
    return CalibrationDocument(
        schema_version=data["schema_version"],
        robot=data["robot"],
        robot_revision=data["robot_revision"],
        created_utc=data["created_utc"],
        source=data["source"],
        joints=[JointCalibration(**joint) for joint in data["joints"]],
    )


def load_calibration(path: Path) -> CalibrationDocument:
    return document_from_dict(json.loads(path.read_text()))


def write_calibration(path: Path, document: CalibrationDocument) -> None:
    path.write_text(json.dumps(document_to_dict(document), indent=2) + "\n")


def validate_calibration(document: CalibrationDocument, require_complete: bool = True) -> list[str]:
    errors: list[str] = []

    if document.schema_version != SCHEMA_VERSION:
        errors.append(f"schema_version must be {SCHEMA_VERSION}")

    expected_by_id = {
        motor_id: {"leg": leg, "joint": joint, "bus": bus}
        for motor_id, leg, joint, bus in expected_joints()
    }

    seen: set[int] = set()
    for item in document.joints:
        if item.motor_id in seen:
            errors.append(f"duplicate motor_id {item.motor_id}")
            continue
        seen.add(item.motor_id)

        expected = expected_by_id.get(item.motor_id)
        if expected is None:
            errors.append(f"unexpected motor_id {item.motor_id}")
            continue

        for field in ("leg", "joint", "bus"):
            if getattr(item, field) != expected[field]:
                errors.append(
                    f"motor {item.motor_id} {field}={getattr(item, field)!r}; "
                    f"expected {expected[field]!r}"
                )

        if item.sign not in (-1, 1):
            errors.append(f"motor {item.motor_id} sign must be -1 or 1")

        numeric_fields = (
            "zero_offset_rad",
            "min_position_rad",
            "max_position_rad",
            "park_position_rad",
        )
        values = {field: getattr(item, field) for field in numeric_fields}
        if require_complete:
            for field, value in values.items():
                if value is None:
                    errors.append(f"motor {item.motor_id} {field} is required")

        minimum = item.min_position_rad
        maximum = item.max_position_rad
        park = item.park_position_rad
        if minimum is not None and maximum is not None and minimum >= maximum:
            errors.append(f"motor {item.motor_id} min_position_rad must be < max_position_rad")
        if minimum is not None and maximum is not None and park is not None:
            if not minimum <= park <= maximum:
                errors.append(f"motor {item.motor_id} park_position_rad outside limits")

    missing = sorted(set(expected_by_id) - seen)
    for motor_id in missing:
        errors.append(f"missing motor_id {motor_id}")

    return errors


def is_valid_calibration(path: Path) -> bool:
    return not validate_calibration(load_calibration(path))


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate or create a TOCATL calibration JSON file")
    parser.add_argument("path", nargs="?", type=Path, help="Calibration JSON file to validate")
    parser.add_argument("--write-template", type=Path, help="Write an incomplete template JSON file")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)

    if args.write_template:
        write_calibration(args.write_template, calibration_template())
        print(f"Wrote incomplete calibration template: {args.write_template}")
        print("Fill and validate it before using it as an arming precondition.")
        return 0

    if not args.path:
        print(json.dumps(document_to_dict(calibration_template()), indent=2))
        return 0

    errors = validate_calibration(load_calibration(args.path))
    if errors:
        print(f"INVALID calibration: {args.path}")
        for error in errors:
            print(f"- {error}")
        return 1

    print(f"Valid calibration: {args.path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
