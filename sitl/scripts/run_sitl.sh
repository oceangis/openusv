#!/bin/bash
# run_sitl.sh - One-command USV SITL launch wrapper
# Works on Git Bash (Windows), WSL, and native Linux.
set -e

###############################################################################
# Usage
###############################################################################
FRAME="${1:?Usage: $0 <frame-name> [sim_vehicle.py options]
Available frames:
  usv-diff          Differential-thrust twin-motor
  usv-diff-bow      Diff-thrust + bow thruster
  usv-xframe        X-frame 4-motor
  usv-wingsail-flap Wing-sail with flap
  usv-wingsail      Wing-sail without flap
  usv-rudder        Single-motor + rudder}"
shift

###############################################################################
# Resolve paths
###############################################################################
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Find ArduPilot root -- check AP_ROOT env var, then default relative path
if [ -n "$AP_ROOT" ] && [ -d "$AP_ROOT" ]; then
    AP_DIR="$AP_ROOT"
else
    AP_DIR="$(cd "$PROJECT_ROOT/../../ardupilot-master" 2>/dev/null && pwd)" || true
fi

if [ -z "$AP_DIR" ] || [ ! -d "$AP_DIR" ]; then
    echo "ERROR: ArduPilot root not found."
    echo "Set AP_ROOT or ensure ../../ardupilot-master exists."
    exit 1
fi

SIM_VEHICLE="$AP_DIR/Tools/autotest/sim_vehicle.py"
VEHICLEINFO="$AP_DIR/Tools/autotest/pysim/vehicleinfo.py"

###############################################################################
# Verify setup was done
###############################################################################
if ! grep -q '"usv-diff"' "$VEHICLEINFO" 2>/dev/null; then
    echo "ERROR: USV models not registered in vehicleinfo.py."
    echo "Run setup first:  $PROJECT_ROOT/sitl/scripts/setup_sitl.sh"
    exit 1
fi

if [ ! -f "$SIM_VEHICLE" ]; then
    echo "ERROR: sim_vehicle.py not found at $SIM_VEHICLE"
    exit 1
fi

###############################################################################
# Default home: Shenzhen Bay
###############################################################################
HOME_LOC="${SITL_HOME:-22.5024,113.9150,0,270}"

###############################################################################
# Launch
###############################################################################
echo "=============================================="
echo " USV SITL Launcher"
echo "=============================================="
echo "Frame   : $FRAME"
echo "Home    : $HOME_LOC"
echo "ArduPilot: $AP_DIR"
echo "Extra args: $*"
echo "=============================================="
echo ""

exec python3 "$SIM_VEHICLE" \
    -v Rover \
    -f "$FRAME" \
    -l "$HOME_LOC" \
    "$@"
