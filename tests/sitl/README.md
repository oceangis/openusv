# USV Sailboat SITL Test Framework

Python-based automated testing framework for the USV wingsail control system.

## Quick Start

```bash
# From tests directory
cd tests

# Run all tests
python run_all_tests.py

# Run quick test subset
python run_all_tests.py --quick

# Run specific test categories
python run_all_tests.py --wingsail   # Wingsail mode comparison
python run_all_tests.py --storm      # Storm stability tests
python run_all_tests.py --upwind     # Upwind tacking tests
```

## Module Structure

```
sitl/
├── __init__.py              # Package exports
├── sailboat_sim.py          # Pure Python physics simulator
├── test_base.py             # Base test class & TestResult
├── test_runner.py           # Test execution & reporting
├── test_upwind_tacking.py   # Upwind/tacking tests
├── test_wingsail_modes.py   # Wingsail mode comparison
└── test_storm_stability.py  # Storm condition tests
```

## Test Categories

### 1. Upwind Tacking Tests (`test_upwind_tacking.py`)

- **UpwindTackingTest**: Time-based tacking
- **CrossTrackTackingTest**: Cross-track error based tacking

### 2. Wingsail Mode Comparison (`test_wingsail_modes.py`)

Compares ROTATION, FLAP, and FREE modes on:
- Beam reach
- Upwind (close-hauled)
- Downwind (running)

### 3. Storm Stability (`test_storm_stability.py`)

- **GaleConditionsTest**: Beaufort 8 survival
- **VariableWindTest**: Wind shift adaptation
- **LowWindMotorAssistTest**: Motor assist activation

## Writing Custom Tests

```python
from sitl import SailboatTestBase, WingsailType, BeaufortScale

class MyTest(SailboatTestBase):
    def __init__(self):
        super().__init__(name="My Test", description="...")
        self.duration = 60.0
        self.wingsail_type = WingsailType.WINGSAIL_FLAP

    def setup(self):
        self.sim.init(self.wingsail_type)
        self.sim.set_beaufort(BeaufortScale.MODERATE_BREEZE)

    def control_loop(self, state, time_s):
        # Return (rudder, wingsail, throttle)
        return 0.0, 0.5, 0.0

    def analyze(self):
        super().analyze()
        self.result.add_metric("my_metric", 1.0, "units")
        return self.result
```

## Physics Simulator

`sailboat_sim.py` implements:
- Three wingsail aerodynamic models
- Hull hydrodynamics
- Wave-induced motion
- Tide/current effects

Matches the C++ implementation in `simulation/SIM_Sailboat_USV.cpp`.
