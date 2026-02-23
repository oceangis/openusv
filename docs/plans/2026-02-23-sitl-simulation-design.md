# SITL Simulation System Design - ESP32-S3 USV

Date: 2026-02-23
Status: Approved

## Goal

Build a full SITL (Software In The Loop) simulation system for the ESP32-S3 USV project, supporting 6 boat types. All simulation code lives in this project, injected into `ardupilot-master` via symlinks + minimal patches. Enables development, mission testing, formation control, and algorithm validation without hardware.

## Boat Types

| # | Frame Name | Description | Servo Mapping |
|---|-----------|-------------|---------------|
| 1 | `usv-diff` | Differential thrust (skid steering) | S1=Left, S2=Right |
| 2 | `usv-diff-bow` | Differential + bow thruster | S1=Left, S2=Right, S3=Lateral |
| 3 | `usv-xframe` | X-frame quad thruster | S1-S4=Motors |
| 4 | `usv-wingsail-flap` | Wingsail with flap + rudder | S1=Rudder, S3=Wingsail, S4=Flap |
| 5 | `usv-wingsail` | Wingsail (no flap) + rudder | S1=Rudder, S3=Wingsail |
| 6 | `usv-rudder` | Single thruster + rudder | S1=Rudder, S3=Throttle |

## Architecture

```
esp32s3rover/
├── sitl/
│   ├── models/                         # C++ physics models
│   │   ├── SIM_USV_Base.h/cpp          # Common water dynamics (waves, drag, current)
│   │   ├── SIM_USV_DiffThrust.h/cpp    # #1 Differential thrust
│   │   ├── SIM_USV_DiffBow.h/cpp       # #2 Diff + bow thruster
│   │   ├── SIM_USV_XFrame.h/cpp        # #3 X-frame quad
│   │   ├── SIM_USV_Wingsail.h/cpp      # #4 #5 Wingsail (flap flag)
│   │   └── SIM_USV_Rudder.h/cpp        # #6 Single thruster + rudder
│   ├── params/
│   │   ├── usv-diff.parm
│   │   ├── usv-diff-bow.parm
│   │   ├── usv-xframe.parm
│   │   ├── usv-wingsail-flap.parm
│   │   ├── usv-wingsail.parm
│   │   └── usv-rudder.parm
│   ├── scripts/
│   │   ├── setup_sitl.sh               # Deploy: symlink + patch ardupilot-master
│   │   └── run_sitl.sh                 # Launch: one-command SITL startup
│   ├── patches/
│   │   ├── sitl_cmdline.patch          # Register 6 models in model_constructors[]
│   │   └── vehicleinfo.patch           # Register 6 frames in Python launcher
│   └── tests/
│       ├── test_usv_diff.py            # Diff thrust mission tests
│       ├── test_usv_formation.py       # Multi-vehicle formation tests
│       └── test_usv_wingsail.py        # Wingsail tacking/VMG tests
```

## Class Hierarchy

```
Aircraft (ArduPilot base)
└── SIM_USV_Base (our base)              # Water dynamics: waves, drag, current, buoyancy
    ├── SIM_USV_DiffThrust               # skid_steering = true
    │   └── SIM_USV_DiffBow              # + lateral thruster force
    ├── SIM_USV_XFrame                   # 4-motor vector sum
    ├── SIM_USV_Wingsail                 # Wind aerodynamics + rudder
    │   ├── mode: flap (S4 = flap)       #   wingsail body + trailing edge flap
    │   └── mode: no-flap (S4 unused)    #   wingsail body rotation only
    └── SIM_USV_Rudder                   # Single motor + rudder angle
```

## SIM_USV_Base - Common Water Physics

Shared by all 6 boat types:

- **Hull drag**: Quadratic resistance F_drag = -0.5 * Cd * rho * A * v^2
- **Wave effects**: Roll/pitch oscillation + heave (Douglas sea state 0-9)
- **Tide/current**: Constant velocity offset in earth frame
- **Water constraint**: Vehicles stay on water surface (no vertical DOF beyond waves)
- **Heading dynamics**: Yaw rate from applied torque, damped by water resistance

Reuses proven physics from ArduPilot `SIM_Sailboat.cpp` (wave model, water velocity frame) and our existing `simulation/SIM_Sailboat_USV.cpp` (Beaufort scale, Douglas sea state).

## Physics Model Details

### 1. usv-diff (Differential Thrust)

```
Force_x = (S1 + S2) * max_thrust
Torque_z = (S2 - S1) * max_thrust * arm_length
```

- Inherits: SIM_USV_Base
- Parameters: max_thrust, arm_length (distance between motors), mass
- ArduPilot config: FRAME_CLASS=2, SERVO1_FUNCTION=73, SERVO2_FUNCTION=74

### 2. usv-diff-bow (Differential + Bow Thruster)

```
Force_x = (S1 + S2) * max_thrust
Force_y = S3 * bow_thrust                     # lateral force at bow
Torque_z = (S2 - S1) * max_thrust * arm_y
          + S3 * bow_thrust * arm_x_bow       # bow thruster moment arm
```

- Inherits: SIM_USV_DiffThrust (adds lateral force)
- Parameters: + bow_thrust, arm_x_bow (bow thruster distance from CG)
- ArduPilot config: + SERVO3_FUNCTION=75 (ThrottleLateral)

### 3. usv-xframe (X-Frame Quad Thruster)

```
        M1(FL)     M2(FR)
           \       /
            \     /
             [CG]
            /     \
           /       \
        M3(BL)     M4(BR)

Force_x  = (M1+M2+M3+M4) * cos(mount_angle) * thrust_per_motor
Force_y  = (-M1+M2-M3+M4) * sin(mount_angle) * thrust_per_motor
Torque_z = (-M1+M2+M3-M4) * arm * thrust_per_motor
```

- Inherits: SIM_USV_Base
- Parameters: mount_angle (45 deg typical), arm, thrust_per_motor
- ArduPilot config: FRAME_CLASS=2, SERVO1-4_FUNCTION mapped per motor
- Full omnidirectional movement capability

### 4 & 5. usv-wingsail / usv-wingsail-flap (Wingsail)

```
Apparent_wind = True_wind - Boat_velocity
AoA = Wingsail_angle - Apparent_wind_angle     (+ flap offset if applicable)
F_lift = 0.5 * rho * chord * span * Cl(AoA)
F_drag = 0.5 * rho * chord * span * Cd(AoA)
Rudder_force = 0.5 * rho * rudder_area * Cd_rudder * v^2 * sin(rudder_angle)
```

- Inherits: SIM_USV_Base
- With flap: Cl/Cd curves shifted by flap deflection (higher Cl_max)
- Without flap: Cl/Cd from wingsail rotation angle only
- Aerodynamic tables: 18-point interpolation (from existing SIM_Sailboat_USV)
- ArduPilot config: WNDVN_TYPE for wind sensing, SERVO mappings per variant

### 6. usv-rudder (Single Thruster + Rudder)

```
Force_x = S3 * max_thrust
Rudder_force_y = k * v^2 * sin(rudder_angle)
Torque_z = Rudder_force_y * rudder_arm
```

- Inherits: SIM_USV_Base
- Parameters: max_thrust, rudder_area, rudder_arm
- Simplest model, traditional boat layout

## Upstream Integration (Minimal Patch)

### Patch 1: SITL_cmdline.cpp (6 lines added)

```cpp
#include <SITL/SIM_USV_DiffThrust.h>
#include <SITL/SIM_USV_DiffBow.h>
#include <SITL/SIM_USV_XFrame.h>
#include <SITL/SIM_USV_Wingsail.h>
#include <SITL/SIM_USV_Rudder.h>

// In model_constructors[]:
{ "usv-diff",           USV_DiffThrust::create },
{ "usv-diff-bow",       USV_DiffBow::create },
{ "usv-xframe",         USV_XFrame::create },
{ "usv-wingsail-flap",  USV_Wingsail::create },
{ "usv-wingsail",       USV_Wingsail::create },
{ "usv-rudder",         USV_Rudder::create },
```

### Patch 2: vehicleinfo.py (6 frame entries)

```python
"usv-diff": {
    "waf_target": "bin/ardurover",
    "default_params_filename": ["default_params/rover.parm",
                                "default_params/usv-diff.parm"],
},
# ... same pattern for all 6 frames
```

### setup_sitl.sh Script

```bash
#!/bin/bash
# 1. Symlink model source files into ardupilot-master/libraries/SITL/
# 2. Symlink param files into ardupilot-master/Tools/autotest/default_params/
# 3. Apply patches to SITL_cmdline.cpp and vehicleinfo.py
# 4. Build: ./waf configure --board sitl && ./waf rover
```

## Usage

```bash
# Setup (once)
./sitl/scripts/setup_sitl.sh

# Run single boat
./sitl/scripts/run_sitl.sh usv-diff

# Run formation (3 boats)
./sitl/scripts/run_sitl.sh usv-diff --instance 0 &
./sitl/scripts/run_sitl.sh usv-diff --instance 1 &
./sitl/scripts/run_sitl.sh usv-diff --instance 2 &

# Connect QGC to 127.0.0.1:5760 (boat 0), :5770 (boat 1), :5780 (boat 2)

# Run wingsail with flap
./sitl/scripts/run_sitl.sh usv-wingsail-flap

# Run automated tests
python sitl/tests/test_usv_diff.py
python sitl/tests/test_usv_formation.py
```

## Environment Simulation

Available through SITL parameters (SIM_* prefix):

- **Wind**: SIM_WIND_SPD, SIM_WIND_DIR, SIM_WIND_TURB (critical for wingsail)
- **Waves**: Configurable via sea state in SIM_USV_Base
- **Current**: SIM_TIDE_SPEED, SIM_TIDE_DIR
- **Sensor noise**: SIM_GPS_GLITCH, SIM_MAG_OFS, SIM_ACC_BIAS

## Implementation Priority

1. **Phase 1**: SIM_USV_Base + SIM_USV_DiffThrust + setup_sitl.sh (MVP)
2. **Phase 2**: SIM_USV_DiffBow + SIM_USV_Rudder
3. **Phase 3**: SIM_USV_Wingsail (both variants)
4. **Phase 4**: SIM_USV_XFrame
5. **Phase 5**: Formation tests + automated CI

## Success Criteria

- `run_sitl.sh usv-diff` launches in <30s, connects to QGC
- AUTO mission completes waypoint circuit
- RTL failsafe triggers on GCS disconnect
- Formation: 3 boats hold relative position
- Wingsail: VMG positive when tacking upwind
