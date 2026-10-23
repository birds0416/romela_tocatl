#!/usr/bin/env bash
set -u

skip_service_check=0
if [ "${1:-}" = "--skip-service-check" ]; then
  skip_service_check=1
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root" || exit 1

export PYTHONPATH="src/tocatl_control:src/tocatl_diagnostics:src/tocatl_kinematics:src/tocatl_gait"
failures=0

section() {
  printf "\n== %s ==\n" "$1"
}

record_failure() {
  printf "FAIL: %s\n" "$1" >&2
  failures=$((failures + 1))
}

run_required() {
  section "$1"
  shift
  "$@"
  status=$?
  if [ "$status" -ne 0 ]; then
    record_failure "$* exited $status"
  fi
}

section "Safety notice"
cat <<'NOTICE'
This script is dry-run only:
- do not pass --live to telemetry tools
- do not run recovered legacy walking scripts
- keep motor power disconnected unless a reviewed hardware test says otherwise
NOTICE

if [ "$skip_service_check" -eq 0 ]; then
  section "Legacy service state"
  if ! command -v systemctl >/dev/null 2>&1; then
    record_failure "systemctl not found; run on the Pi or use --skip-service-check for laptop-only checks"
  else
    enabled="$(systemctl --user is-enabled tocatl.service 2>/dev/null || true)"
    active="$(systemctl --user is-active tocatl.service 2>/dev/null || true)"
    printf "tocatl.service enabled: %s\n" "$enabled"
    printf "tocatl.service active:  %s\n" "$active"
    if [ "$enabled" != "disabled" ]; then
      record_failure "tocatl.service must remain disabled"
    fi
    if [ "$active" != "inactive" ]; then
      record_failure "tocatl.service must remain inactive"
    fi
  fi
else
  section "Legacy service state"
  printf "Skipped service check by request. Use this only for laptop/local verification.\n"
fi

run_required "Unit tests" python3 -m unittest discover -s test -v
run_required "Motor map dry run" python3 -m tocatl_diagnostics.motor_map
run_required "Legacy geometry reference dry run" python3 -m tocatl_kinematics.legacy_geometry
run_required "Legacy gait reference dry run" python3 -m tocatl_gait.legacy_reference
run_required "Telemetry schema dry run" python3 -m tocatl_diagnostics.cybergear_telemetry

section "Calibration template negative check"
tmp_output="$(mktemp)"
python3 -m tocatl_control.calibration config/calibration-template.json >"$tmp_output" 2>&1
calibration_status=$?
sed -n '1,12p' "$tmp_output"
rm -f "$tmp_output"
if [ "$calibration_status" -eq 0 ]; then
  record_failure "calibration template unexpectedly validated"
else
  printf "PASS: calibration template is invalid by design.\n"
fi

section "Result"
if [ "$failures" -eq 0 ]; then
  printf "PASS: dry-run checks completed without failures.\n"
  exit 0
fi

printf "FAIL: %s dry-run check(s) failed.\n" "$failures" >&2
exit 1
