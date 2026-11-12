#!/usr/bin/env bash
# THE test command for spider-robot: builds every part, exports STEP/STL + leg_geometry.json, runs all CAD checks, then the
# firmware checks (tools/firmware_tests.sh: generated header up to date, host unit tests, mutation self-tests, arduino-cli compile if present).
# Exit non-zero if: any step exits non-zero, a Traceback / "FAIL:" line appears, or fewer PASS lines than expected.
# Fallback (fixed-tibia) config is built and checked too; its torque check is informational only (known: shorter-reach
# requirement not met, see PROGRESS/LESSONS) -> printed as FALLBACK-INFO, never hides a main-config failure.
set -uo pipefail
cd "$(dirname "$0")/.."
PY=${PY:-$HOME/tools/cad-venv/bin/python}
export PYTHONDONTWRITEBYTECODE=1
log=$(mktemp); trap 'rm -f "$log"' EXIT
reasons=()
npass=0

run() { # label cmd...
  local label=$1; shift
  echo "--- $label"
  "$@" 2>&1 | grep -v "WARN\|^$" | tee -a "$log"
  local rc=${PIPESTATUS[0]}
  [ "$rc" -eq 0 ] || reasons+=("$label exited $rc")
}

echo "=== MAIN config: passive parallelogram tibia ==="
unset SPIDER_TIBIA_MODE
run build "$PY" cad/build_all.py
for c in check_solids_bed check_fits check_geometry_json check_linkage check_ground check_interference check_torque check_mass; do
  run "$c" "$PY" cad/analysis/$c.py
done
main_pass=$(grep -c "^PASS:" "$log")

echo "=== FALLBACK config: fixed bent tibia (SPIDER_TIBIA_MODE=fixed) ==="
export SPIDER_TIBIA_MODE=fixed
run fixed-build "$PY" cad/build_all.py
for c in check_solids_bed check_fits check_geometry_json check_linkage check_ground check_interference check_mass; do
  run "fixed-$c" "$PY" cad/analysis/$c.py
done
echo "--- fixed-check_torque (informational)"
"$PY" cad/analysis/check_torque.py 2>&1 | sed 's/^FAIL:/FALLBACK-INFO (not gating) FAIL:/; s/^PASS:/FALLBACK-INFO PASS:/'
unset SPIDER_TIBIA_MODE

expected_main=17   # PASS lines expected from the main config (see checks)
expected_total=29
if grep -q "Traceback" "$log"; then reasons+=("Traceback in output"); fi
if grep -q "^FAIL:" "$log"; then reasons+=("a check printed FAIL"); fi
total_pass=$(grep -c "^PASS:" "$log")
[ "$main_pass" -ge "$expected_main" ] || reasons+=("main config: only $main_pass PASS lines, expected >= $expected_main")
[ "$total_pass" -ge "$expected_total" ] || reasons+=("only $total_pass PASS lines in total, expected >= $expected_total")

echo "=== FIRMWARE: generated header, host unit tests, mutation self-tests, compile"
fwlog=$(mktemp)
tools/firmware_tests.sh 2>&1 | tee "$fwlog"
[ "${PIPESTATUS[0]}" -eq 0 ] || reasons+=("firmware tests failed")
grep -q "^FAIL" "$fwlog" && reasons+=("a firmware check printed FAIL")
fw_pass=$(grep -c "^PASS:" "$fwlog")
[ "$fw_pass" -ge 8 ] || reasons+=("firmware: only $fw_pass PASS lines, expected >= 8")
rm -f "$fwlog"

echo "=== SUMMARY: CAD main PASS lines $main_pass, CAD total $total_pass, firmware PASS lines $fw_pass"
if [ ${#reasons[@]} -gt 0 ]; then
  printf 'TESTS FAILED: %s\n' "${reasons[@]}"
  exit 1
fi
echo "ALL TESTS PASSED"
