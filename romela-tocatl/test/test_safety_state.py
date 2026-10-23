from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).parents[1] / "src" / "tocatl_control"
sys.path.insert(0, str(PACKAGE_ROOT))

from tocatl_control.safety_state import SafetyInputs, SafetyState, SafetyStateMachine  # noqa: E402


class SafetyStateMachineTests(unittest.TestCase):
    def test_startup_does_not_arm_without_explicit_operator_request(self):
        machine = SafetyStateMachine()
        machine.start_self_test()
        machine.self_test_passed()
        machine.update_readiness(SafetyInputs(
            telemetry_fresh=True,
            calibration_valid=True,
            physical_estop_ok=True,
        ))
        self.assertEqual(machine.state, SafetyState.READY)

    def test_ready_requires_operator_arm_request_to_enter_armed(self):
        machine = SafetyStateMachine()
        machine.start_self_test()
        machine.self_test_passed()
        ready = SafetyInputs(
            telemetry_fresh=True,
            calibration_valid=True,
            physical_estop_ok=True,
        )
        machine.update_readiness(ready)
        machine.update_readiness(SafetyInputs(
            telemetry_fresh=True,
            calibration_valid=True,
            physical_estop_ok=True,
            operator_arm_requested=True,
        ))
        self.assertEqual(machine.state, SafetyState.ARMED)

    def test_estop_not_ready_forces_disarmed_and_not_auto_restart(self):
        machine = SafetyStateMachine(state=SafetyState.READY)
        machine.update_readiness(SafetyInputs(
            telemetry_fresh=True,
            calibration_valid=True,
            physical_estop_ok=False,
            operator_arm_requested=True,
        ))
        self.assertEqual(machine.state, SafetyState.DISARMED)

        machine.update_readiness(SafetyInputs(
            telemetry_fresh=True,
            calibration_valid=True,
            physical_estop_ok=True,
        ))
        self.assertEqual(machine.state, SafetyState.DISARMED)

    def test_stale_telemetry_trips_fault_while_armed(self):
        machine = SafetyStateMachine(state=SafetyState.ARMED)
        machine.update_readiness(SafetyInputs(
            telemetry_fresh=False,
            calibration_valid=True,
            physical_estop_ok=True,
        ))
        self.assertEqual(machine.state, SafetyState.FAULT)
        self.assertEqual(machine.fault_reason, "telemetry stale while armed")

    def test_fault_requires_manual_reset(self):
        machine = SafetyStateMachine(state=SafetyState.ARMED)
        machine.trip("test fault")
        machine.update_readiness(SafetyInputs(
            telemetry_fresh=True,
            calibration_valid=True,
            physical_estop_ok=True,
            operator_arm_requested=True,
        ))
        self.assertEqual(machine.state, SafetyState.FAULT)
        machine.reset_fault()
        self.assertEqual(machine.state, SafetyState.DISARMED)


if __name__ == "__main__":
    unittest.main()
