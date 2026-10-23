"""Pure safety/arming state machine for TOCATL.

This module intentionally has no ROS, CAN, or motor-driver dependency. It is
safe to unit test on a laptop and should be the single place where controller
arming rules are made explicit.
"""

from dataclasses import dataclass, field
from enum import Enum


class SafetyState(str, Enum):
    DISARMED = "DISARMED"
    SELF_TEST = "SELF_TEST"
    CALIBRATION_CHECK = "CALIBRATION_CHECK"
    READY = "READY"
    ARMED = "ARMED"
    FAULT = "FAULT"


@dataclass
class SafetyInputs:
    """Controller-side preconditions for arming.

    These are software readiness checks only. They do not replace the physical
    motor-power E-stop, fuse, contactor, or stand/restraint.
    """

    telemetry_fresh: bool = False
    calibration_valid: bool = False
    operator_arm_requested: bool = False
    physical_estop_ok: bool = False
    no_active_faults: bool = True


@dataclass
class SafetyStateMachine:
    state: SafetyState = SafetyState.DISARMED
    fault_reason: str | None = None
    history: list[str] = field(default_factory=list)

    def start_self_test(self) -> None:
        if self.state == SafetyState.DISARMED:
            self._transition(SafetyState.SELF_TEST, "self-test started")

    def self_test_passed(self) -> None:
        if self.state == SafetyState.SELF_TEST:
            self._transition(SafetyState.CALIBRATION_CHECK, "self-test passed")

    def update_readiness(self, inputs: SafetyInputs) -> None:
        """Advance only through non-motion readiness states.

        Releasing an E-stop or restoring telemetry never arms the robot by
        itself. `operator_arm_requested` must be true while already READY.
        """

        if self.state == SafetyState.FAULT:
            return
        if not inputs.no_active_faults:
            self.trip("active fault reported")
            return
        if not inputs.physical_estop_ok:
            self._transition(SafetyState.DISARMED, "physical E-stop not ready")
            return
        if self.state == SafetyState.CALIBRATION_CHECK:
            if inputs.telemetry_fresh and inputs.calibration_valid:
                self._transition(SafetyState.READY, "telemetry and calibration valid")
        elif self.state == SafetyState.READY and inputs.operator_arm_requested:
            self._transition(SafetyState.ARMED, "operator armed")
        elif self.state == SafetyState.ARMED:
            if not inputs.telemetry_fresh:
                self.trip("telemetry stale while armed")
            elif not inputs.calibration_valid:
                self.trip("calibration invalid while armed")

    def disarm(self, reason: str = "operator disarmed") -> None:
        self._transition(SafetyState.DISARMED, reason)

    def trip(self, reason: str) -> None:
        self.fault_reason = reason
        self._transition(SafetyState.FAULT, reason)

    def reset_fault(self) -> None:
        if self.state == SafetyState.FAULT:
            self.fault_reason = None
            self._transition(SafetyState.DISARMED, "fault reset")

    def _transition(self, new_state: SafetyState, reason: str) -> None:
        if new_state != self.state:
            self.history.append(f"{self.state.value}->{new_state.value}: {reason}")
            self.state = new_state
