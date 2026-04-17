#!/bin/bash
# cosys_scenario_check.sh
# Usage: ./cosys_scenario_check.sh <scenario_config.json>
# Example: ./cosys_scenario_check.sh config/scenarios/29_cosys_perception.json

set -e

SCENARIO_CONFIG="$1"
if [[ -z "$SCENARIO_CONFIG" ]]; then
  echo "Usage: $0 <scenario_config.json>"
  exit 1
fi

# 1. Clean out old run directories
echo "[1/5] Cleaning old scenario logs..."
rm -rf ~/Projects/companion_software_stack/drone_logs/scenarios_cosys/*

# 2. Verify AirSim settings.json
SETTINGS=~/Documents/AirSim/settings.json
echo "[2/5] Checking AirSim settings.json..."
grep -q '"mission_cam"' "$SETTINGS" || { echo "[ERROR] mission_cam missing in settings.json"; exit 1; }
grep -q '"depth_cam"' "$SETTINGS" || { echo "[ERROR] depth_cam missing in settings.json"; exit 1; }
grep -q '"x500"' "$SETTINGS" || { echo "[ERROR] x500 vehicle missing in settings.json"; exit 1; }
echo "[OK] AirSim settings.json looks correct."

# 3. Verify scenario config camera/vehicle names
CAM_ERRORS=0
echo "[3/5] Checking scenario config camera/vehicle names..."
if grep -q '"camera_name":' "$SCENARIO_CONFIG"; then
  BAD=$(grep -v '"camera_name": "mission_cam"' "$SCENARIO_CONFIG" | grep '"camera_name":')
  if [[ -n "$BAD" ]]; then
    echo "[ERROR] Non-matching camera_name(s):"
    echo "$BAD"
    CAM_ERRORS=1
  fi
fi
if grep -q '"vehicle_name":' "$SCENARIO_CONFIG"; then
  BAD=$(grep -v '"vehicle_name": "x500"' "$SCENARIO_CONFIG" | grep '"vehicle_name":')
  if [[ -n "$BAD" ]]; then
    echo "[ERROR] Non-matching vehicle_name(s):"
    echo "$BAD"
    CAM_ERRORS=1
  fi
fi
if grep -q '"camera_name_left":' "$SCENARIO_CONFIG"; then
  BAD=$(grep -v '"camera_name_left": "mission_cam_left"' "$SCENARIO_CONFIG" | grep '"camera_name_left":')
  if [[ -n "$BAD" ]]; then
    echo "[ERROR] Non-matching camera_name_left(s):"
    echo "$BAD"
    CAM_ERRORS=1
  fi
fi
if grep -q '"camera_name_right":' "$SCENARIO_CONFIG"; then
  BAD=$(grep -v '"camera_name_right": "mission_cam_right"' "$SCENARIO_CONFIG" | grep '"camera_name_right":')
  if [[ -n "$BAD" ]]; then
    echo "[ERROR] Non-matching camera_name_right(s):"
    echo "$BAD"
    CAM_ERRORS=1
  fi
fi
if grep -q '"camera_name": "depth_cam"' "$SCENARIO_CONFIG"; then
  echo "[OK] depth_cam present for depth blocks."
fi
if [[ $CAM_ERRORS -eq 1 ]]; then
  echo "[FAIL] Scenario config has camera/vehicle name mismatches."
  exit 1
else
  echo "[OK] Scenario config camera/vehicle names are correct."
fi

# 4. Run the scenario
echo "[4/5] Running scenario..."
./tests/run_scenario_cosys.sh "$SCENARIO_CONFIG" --gui --verbose || true

# 5. If crash, print last 40 lines of UE5 log and cosys_ue5.log
echo "[5/5] Checking for segfaults or camera errors..."
UE5LOG=$(ls -1t ~/.config/Epic/UnrealEngine/5.4/Saved/Logs/Blocks.log 2>/dev/null | head -1)
if [[ -n "$UE5LOG" ]]; then
  echo "--- Last 40 lines of UE5 log ---"
  tail -40 "$UE5LOG"
fi
LATEST_RUN=$(ls -1dt ~/Projects/companion_software_stack/drone_logs/scenarios_cosys/cosys_perception/* 2>/dev/null | head -1)
if [[ -n "$LATEST_RUN" ]]; then
  if [[ -f "$LATEST_RUN/cosys_ue5.log" ]]; then
    echo "--- Last 40 lines of cosys_ue5.log ---"
    tail -40 "$LATEST_RUN/cosys_ue5.log"
  fi
fi

echo "[DONE] If you see a camera_name not found error above, fix it in both settings.json and scenario config."
