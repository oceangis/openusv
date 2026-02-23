#!/bin/bash
# setup_sitl.sh - Deploy USV SITL models into ardupilot-master
# Works on Git Bash (Windows), WSL, and native Linux.
set -e

###############################################################################
# Resolve paths
###############################################################################
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

AP_ROOT="${1:-$(cd "$PROJECT_ROOT/../../ardupilot-master" 2>/dev/null && pwd)}"
if [ -z "$AP_ROOT" ] || [ ! -d "$AP_ROOT" ]; then
    echo "ERROR: ArduPilot root not found."
    echo "Usage: $0 [path_to_ardupilot]"
    echo "  Default: ../../ardupilot-master relative to project root"
    exit 1
fi

SITL_MODELS_SRC="$PROJECT_ROOT/sitl/models"
SITL_PARAMS_SRC="$PROJECT_ROOT/sitl/params"
AP_SITL_LIB="$AP_ROOT/libraries/SITL"
AP_DEFAULT_PARAMS="$AP_ROOT/Tools/autotest/default_params"
AP_CMDLINE="$AP_ROOT/libraries/AP_HAL_SITL/SITL_cmdline.cpp"
AP_VEHICLEINFO="$AP_ROOT/Tools/autotest/pysim/vehicleinfo.py"

echo "=============================================="
echo " USV SITL Setup"
echo "=============================================="
echo "Project root : $PROJECT_ROOT"
echo "ArduPilot root: $AP_ROOT"
echo ""

###############################################################################
# Step 1 - Symlink model sources into libraries/SITL/
###############################################################################
echo "[1/4] Symlinking model sources into $AP_SITL_LIB ..."

count=0
for f in "$SITL_MODELS_SRC"/SIM_USV_*.h "$SITL_MODELS_SRC"/SIM_USV_*.cpp; do
    [ -f "$f" ] || continue
    fname="$(basename "$f")"
    ln -sf "$f" "$AP_SITL_LIB/$fname"
    echo "  -> $fname"
    count=$((count + 1))
done
echo "  $count files linked."
echo ""

###############################################################################
# Step 2 - Symlink param files into Tools/autotest/default_params/
###############################################################################
echo "[2/4] Symlinking param files into $AP_DEFAULT_PARAMS ..."

count=0
for f in "$SITL_PARAMS_SRC"/usv-*.parm; do
    [ -f "$f" ] || continue
    fname="$(basename "$f")"
    ln -sf "$f" "$AP_DEFAULT_PARAMS/$fname"
    echo "  -> $fname"
    count=$((count + 1))
done
echo "  $count files linked."
echo ""

###############################################################################
# Step 3 - Patch SITL_cmdline.cpp
###############################################################################
echo "[3/4] Patching SITL_cmdline.cpp ..."

if grep -q 'SIM_USV_DiffThrust' "$AP_CMDLINE"; then
    echo "  Already patched -- skipping."
else
    # --- Add #include lines after #include <SITL/SIM_MotorBoat.h> ---
    INCLUDE_PATCH='#include <SITL/SIM_USV_DiffThrust.h>\n#include <SITL/SIM_USV_DiffBow.h>\n#include <SITL/SIM_USV_XFrame.h>\n#include <SITL/SIM_USV_Wingsail.h>\n#include <SITL/SIM_USV_Rudder.h>'

    sed -i "/#include <SITL\/SIM_MotorBoat.h>/a\\
$INCLUDE_PATCH" "$AP_CMDLINE"

    # --- Add model constructors after { "motorboat", ---
    MODEL_PATCH='    { "usv-diff",           USV_DiffThrust::create },\n    { "usv-diff-bow",       USV_DiffBow::create },\n    { "usv-xframe",         USV_XFrame::create },\n    { "usv-wingsail-flap",  USV_Wingsail::create },\n    { "usv-wingsail",       USV_Wingsail::create },\n    { "usv-rudder",         USV_Rudder::create },'

    sed -i "/{ \"motorboat\",/a\\
$MODEL_PATCH" "$AP_CMDLINE"

    echo "  Patched successfully."
fi
echo ""

###############################################################################
# Step 4 - Patch vehicleinfo.py
###############################################################################
echo "[4/4] Patching vehicleinfo.py ..."

if grep -q '"usv-diff"' "$AP_VEHICLEINFO"; then
    echo "  Already patched -- skipping."
else
    # We insert the USV frames right after the "sailboat-motor" block.
    # The closing of that block is:  },  (followed by "gazebo-rover")
    # We match the unique line: "default_params/sailboat-motor.parm"],
    # and insert after the next  },

    # Use Python for reliable multi-line insertion (sed is fragile here)
    python3 - "$AP_VEHICLEINFO" <<'PYEOF'
import sys

fpath = sys.argv[1]
with open(fpath, 'r') as f:
    lines = f.readlines()

# Find the sailboat-motor closing brace line
insert_idx = None
for i, line in enumerate(lines):
    if 'sailboat-motor.parm' in line:
        # Find the next '},' after this line
        for j in range(i + 1, len(lines)):
            if lines[j].strip() == '},':
                insert_idx = j + 1
                break
        break

if insert_idx is None:
    print("ERROR: Could not find sailboat-motor block in vehicleinfo.py")
    sys.exit(1)

new_frames = '''\
            "usv-diff": {
                "waf_target": "bin/ardurover",
                "default_params_filename": ["default_params/rover.parm",
                                            "default_params/usv-diff.parm"],
            },
            "usv-diff-bow": {
                "waf_target": "bin/ardurover",
                "default_params_filename": ["default_params/rover.parm",
                                            "default_params/usv-diff-bow.parm"],
            },
            "usv-xframe": {
                "waf_target": "bin/ardurover",
                "default_params_filename": ["default_params/rover.parm",
                                            "default_params/usv-xframe.parm"],
            },
            "usv-wingsail-flap": {
                "waf_target": "bin/ardurover",
                "default_params_filename": ["default_params/rover.parm",
                                            "default_params/usv-wingsail-flap.parm"],
            },
            "usv-wingsail": {
                "waf_target": "bin/ardurover",
                "default_params_filename": ["default_params/rover.parm",
                                            "default_params/usv-wingsail.parm"],
            },
            "usv-rudder": {
                "waf_target": "bin/ardurover",
                "default_params_filename": ["default_params/rover.parm",
                                            "default_params/usv-rudder.parm"],
            },
'''

new_lines = lines[:insert_idx] + [new_frames] + lines[insert_idx:]
with open(fpath, 'w') as f:
    f.writelines(new_lines)

print("  Patched successfully.")
PYEOF
fi
echo ""

###############################################################################
# Done
###############################################################################
echo "=============================================="
echo " Setup complete!"
echo "=============================================="
echo ""
echo "Next steps:"
echo "  cd $AP_ROOT"
echo "  ./waf configure --board sitl"
echo "  ./waf rover"
echo ""
echo "Then launch with:"
echo "  ./sitl/scripts/run_sitl.sh usv-diff"
echo ""
