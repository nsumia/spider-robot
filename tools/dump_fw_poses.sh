#!/usr/bin/env bash
# Build + run the host pose dumper -> cad/out/fw_poses.json (poses the firmware gaits command; consumed by the CAD checks).
set -euo pipefail
cd "$(dirname "$0")/.."
w=$(mktemp -d); trap 'rm -rf "$w"' EXIT
g++ -std=c++14 -O1 -Wall -Wextra -Ifirmware/spider/src firmware/spider/src/*.cpp firmware/tests/dump_poses.cpp -o "$w/dump"
"$w/dump" cad/out/fw_poses.json
echo "wrote cad/out/fw_poses.json ($(grep -c '"name"' cad/out/fw_poses.json) poses)"
