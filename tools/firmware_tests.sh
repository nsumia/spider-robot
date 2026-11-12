#!/usr/bin/env bash
# Firmware checks: generated header up to date, host unit tests, mutation self-tests (broken gait/IK must FAIL), compile.
# Prints PASS:/FAIL: lines; exits non-zero on any failure.
set -uo pipefail
cd "$(dirname "$0")/.."
SRC=firmware/spider/src
CXX=${CXX:-g++}
CXXFLAGS="-std=c++14 -O1 -Wall -Wextra -Werror"
work=$(mktemp -d); trap 'rm -rf "$work"' EXIT
rc=0

python3 tools/gen_geometry_header.py --check || rc=1

build_run() { # srcdir outbin [extra flags]
  $CXX $CXXFLAGS ${3:-} -I"$1" "$1"/*.cpp firmware/tests/test_spider.cpp -o "$2"
}

echo "--- host unit tests"
if build_run $SRC "$work/t"; then
  "$work/t" | tee "$work/out.txt"; trc=${PIPESTATUS[0]}
  if [ "$trc" -eq 0 ] && grep -q "HOST TESTS PASSED" "$work/out.txt" && ! grep -q "^FAIL" "$work/out.txt"; then
    echo "PASS: host unit tests"
  else echo "FAIL: host unit tests (exit $trc)"; rc=1; fi
else echo "FAIL: host test build"; rc=1; fi

echo "--- mutation self-tests (each broken variant must make the host tests fail)"
mutate() { # name file sed-expr
  local d="$work/mut_$1"; mkdir -p "$d"; cp $SRC/*.cpp $SRC/*.h "$d"/
  sed -i "$3" "$d/$2"
  if cmp -s "$SRC/$2" "$d/$2"; then echo "FAIL: mutation '$1' did not change $2 (stale mutation)"; rc=1; return; fi
  if ! build_run "$d" "$work/m_$1" -Wno-error 2>"$work/m_$1.err"; then echo "FAIL: mutation '$1' does not compile"; cat "$work/m_$1.err" | head -5; rc=1; return; fi
  if "$work/m_$1" >"$work/m_$1.out" 2>&1; then echo "FAIL: mutation '$1' was NOT detected by the tests"; rc=1
  else echo "PASS: mutation '$1' detected ($(grep -c '^FAIL' "$work/m_$1.out") failing checks)"; fi
}
mutate creep-two-legs-swing gait.cpp 's/kCreepOff\[4\] = {0.5f, 0.25f, 0.0f, 0.75f}/kCreepOff[4] = {0.5f, 0.25f, 0.0f, 0.25f}/'
mutate creep-no-weight-shift gait.cpp 's/p.shift_mm = 42.0f/p.shift_mm = 0.0f/'
mutate ik-wrong-yaw-sign kinematics.cpp 's/float psi = wrap180(atan2f(dy, dx)/float psi = -wrap180(atan2f(dy, dx)/'
mutate trot-pairs-out-of-phase gait.cpp 's/kTrotOff\[4\] = {0.0f, 0.5f, 0.0f, 0.5f}/kTrotOff[4] = {0.0f, 0.5f, 0.25f, 0.5f}/'
mutate servo-limit-ignored servo.cpp 's/return clampf(s, mn, mx);/return s + 0.0f * (mn + mx);/'
mutate fall-threshold-45 imu_filter.cpp 's/if (tilt_deg > limit_)/if (tilt_deg > limit_ + 15.0f)/'

echo "--- firmware compile (Arduino Nano Every)"
if command -v arduino-cli >/dev/null 2>&1; then
  if arduino-cli compile --fqbn arduino:megaavr:nona4809 firmware/spider >"$work/ac.txt" 2>&1; then
    tail -5 "$work/ac.txt"; echo "PASS: arduino-cli compile"
  else tail -20 "$work/ac.txt"; echo "FAIL: arduino-cli compile"; rc=1; fi
else
  echo "firmware compile: NOT VERIFIED (no arduino-cli)"
fi
exit $rc
