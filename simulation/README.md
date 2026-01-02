# USV Sailboat Simulation Framework

This directory contains the Software-In-The-Loop (SITL) simulation framework for testing the USV wingsail control algorithms.

## Overview

The simulation provides realistic physics modeling for three wingsail control types:

| Type | Description | Use Case |
|------|-------------|----------|
| `WINGSAIL_ROTATION` | Rudder + wingsail rotation | Direct control of entire wingsail angle |
| `WINGSAIL_FLAP` | Rudder + trailing edge flap | OpenTransat-style, self-aligning wingsail |
| `WINGSAIL_FREE` | Rudder only | Sailbuoy-style, fully self-balancing |

## Directory Structure

```
simulation/
├── SIM_Sailboat_USV.h       # Header file with class definitions
├── SIM_Sailboat_USV.cpp     # Physical model implementation
├── sitl_params/             # Pre-configured test scenarios
│   ├── calm_sea.param       # Beaufort 2-3, ideal conditions
│   ├── moderate_wind.param  # Beaufort 4-5, good sailing
│   ├── storm.param          # Beaufort 8-9, stress testing
│   ├── tide_current.param   # Strong current navigation
│   └── upwind_test.param    # Tacking algorithm testing
└── README.md                # This file
```

## Physical Models

### Aerodynamic Model

The simulation uses lift/drag coefficient curves based on:
- **ROTATION mode**: NACA 0012-like symmetric airfoil
- **FLAP mode**: NACA 23012 with 25% chord trailing edge flap

```
Lift Coefficient (Cl) vs Angle of Attack:
    ^
1.5 |       *
    |      * *
1.0 |     *   *
    |    *     *
0.5 |   *       *
    |  *         *
0.0 +--*-----------*--> AoA
   0  10  20  30  40  (degrees)
```

### Hydrodynamic Model

Hull resistance follows:
```
Drag = 0.5 × ρ_water × V² × S_wetted × Cd
```

Where:
- `ρ_water` = 1025 kg/m³
- `V` = boat speed (m/s)
- `S_wetted` = wetted surface area
- `Cd` = drag coefficient

### Wave Model

Waves are modeled as sinusoidal surface:
```
η(x,t) = A × sin(kx - ωt)
```

The boat responds with:
- Roll/pitch oscillation
- Heave motion (vertical acceleration)
- Modified apparent wind

## Environment Parameters

### Beaufort Wind Scale

| Scale | Speed (m/s) | Description |
|-------|-------------|-------------|
| 0 | < 0.5 | Calm |
| 1 | 0.5 - 1.5 | Light air |
| 2 | 1.6 - 3.3 | Light breeze |
| 3 | 3.4 - 5.5 | Gentle breeze |
| 4 | 5.5 - 7.9 | Moderate breeze |
| 5 | 8.0 - 10.7 | Fresh breeze |
| 6 | 10.8 - 13.8 | Strong breeze |
| 7 | 13.9 - 17.1 | Near gale |
| 8 | 17.2 - 20.7 | Gale |
| 9 | 20.8 - 24.4 | Strong gale |
| 10 | 24.5 - 28.4 | Storm |

### Douglas Sea State Scale

| State | Wave Height (m) | Description |
|-------|-----------------|-------------|
| 0 | 0 | Calm (glassy) |
| 1 | 0 - 0.1 | Calm (rippled) |
| 2 | 0.1 - 0.5 | Smooth |
| 3 | 0.5 - 1.25 | Slight |
| 4 | 1.25 - 2.5 | Moderate |
| 5 | 2.5 - 4.0 | Rough |
| 6 | 4.0 - 6.0 | Very rough |
| 7 | 6.0 - 9.0 | High |
| 8 | 9.0 - 14.0 | Very high |
| 9 | > 14.0 | Phenomenal |

## Usage

### Basic Usage (C++)

```cpp
#include "SIM_Sailboat_USV.h"
using namespace SITL_USV;

int main() {
    SailboatUSV sim;

    // Initialize with FLAP mode
    sim.init(WingsailType::WINGSAIL_FLAP);

    // Set environment
    sim.set_beaufort(BeaufortScale::MODERATE_BREEZE);
    sim.set_sea_state(SeaState::SLIGHT);

    // Or use preset scenario
    sim.set_scenario_upwind();

    // Simulation loop
    float dt = 0.01f;  // 100 Hz
    for (int i = 0; i < 10000; i++) {
        // Set control inputs
        sim.set_rudder_input(0.0f);      // -1 to +1
        sim.set_wingsail_input(0.5f);    // -1 to +1

        // Update simulation
        sim.update(dt);

        // Get state
        const SimState& state = sim.get_state();
        printf("Speed: %.2f m/s, VMG: %.2f m/s, Heel: %.1f deg\n",
               state.velocity_bf.x, state.vmg, state.heel_angle_deg);
    }

    return 0;
}
```

### Loading Parameter Files

```cpp
// In future: add param file loader
// sim.load_params("sitl_params/moderate_wind.param");
```

## Test Scenarios

### 1. Upwind Tacking Test

Tests the tacking algorithm performance:
- Wind from target direction
- Evaluate tack timing and angle
- Measure VMG efficiency

```
Expected behavior:
- Automatic tacking when close-hauled
- Smooth heading transitions
- VMG optimization toward target
```

### 2. Storm Survival Test

Tests stability in extreme conditions:
- High wind (20+ m/s)
- Large waves (4+ m)
- Strong currents

```
Expected behavior:
- Switch to FREE/motor mode
- Maintain stability
- Avoid capsizing
```

### 3. Current Navigation Test

Tests navigation in strong tidal currents:
- 2 m/s cross-current
- Waypoint navigation
- Current compensation

```
Expected behavior:
- Correct heading for current
- Maintain track to waypoint
- Efficient power usage
```

## Integration with ArduPilot SITL

To integrate with ArduPilot SITL:

1. Copy simulation files to `libraries/SITL/`
2. Register vehicle type in `SITL.cpp`
3. Add frame string support in `SIM_Aircraft.cpp`

Example SITL launch:
```bash
sim_vehicle.py -v Rover -f sailboat-usv --console --map
```

## Future Enhancements

- [ ] Phase 2: Python test automation framework
- [ ] Phase 3: GitHub Actions CI integration
- [ ] Phase 4: 3D visualization (Gazebo/JSBSim)
- [ ] Wind vane sensor simulation
- [ ] Multi-hull dynamics
- [ ] Collision detection

## References

- [Forces on Sails](https://en.wikipedia.org/wiki/Forces_on_sails)
- [OpenTransat Project](https://opentransat.org/)
- [Sailbuoy](https://www.sailbuoy.no/)
- [ArduPilot Sailboat](https://ardupilot.org/rover/docs/sailboat-home.html)

## License

This file is free software under the GNU General Public License v3.

## Authors

- OpenUSV Project Contributors
- Based on ArduPilot SIM_Sailboat by Randy Mackay
