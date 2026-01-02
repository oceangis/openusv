"""
Storm Stability Tests

Tests sailboat behavior in extreme weather conditions:
- High wind survival
- Large wave handling
- Capsize prevention
- Motor-assist in low/no wind
"""

import math
from typing import Tuple

from .sailboat_sim import SailboatSimulator, WingsailType, BeaufortScale, SeaState, SimState
from .test_base import SailboatTestBase, TestResult, TestStatus


class GaleConditionsTest(SailboatTestBase):
    """
    Test survival in gale conditions (Beaufort 8)

    Verifies that the boat can maintain stability and not capsize
    in strong winds and rough seas.
    """

    def __init__(self):
        super().__init__(
            name="Gale Conditions Survival",
            description="Tests stability in Beaufort 8 gale with rough seas"
        )
        self.duration = 120.0
        self.wingsail_type = WingsailType.WINGSAIL_FREE  # Self-balancing for safety
        self.beaufort = BeaufortScale.GALE
        self.sea_state = SeaState.ROUGH

        self.max_heel_observed = 0.0
        self.capsize_detected = False

    def setup(self):
        """Setup gale test"""
        self.sim.init(self.wingsail_type)
        self.sim.set_beaufort(self.beaufort)
        self.sim.set_sea_state(self.sea_state)

        # Wind from NW, various wave directions
        self.sim.env.wind_direction_deg = 315.0
        self.sim.env.wave_direction_deg = 300.0  # Slightly different from wind
        self.sim.set_heading(180.0)  # Running somewhat downwind

        self.max_heel_observed = 0.0
        self.capsize_detected = False

    def control_loop(self, state: SimState, time_s: float) -> Tuple[float, float, float]:
        """
        Storm survival control

        Minimize sail area, maintain downwind heading
        """
        # Track maximum heel
        heel = abs(state.heel_angle_deg)
        if heel > self.max_heel_observed:
            self.max_heel_observed = heel

        # Detect potential capsize (> 60 degrees)
        if heel > 60.0:
            self.capsize_detected = True

        # Try to maintain a heading that reduces heeling
        # Run slightly off the wind to reduce apparent wind
        target_heading = self.sim.env.wind_direction_deg + 150.0
        if target_heading >= 360:
            target_heading -= 360

        current_heading = math.degrees(state.yaw_rad)
        if current_heading < 0:
            current_heading += 360.0

        heading_error = target_heading - current_heading
        if heading_error > 180:
            heading_error -= 360
        elif heading_error < -180:
            heading_error += 360

        # Gentle steering in storm
        rudder = heading_error * 0.01
        rudder = max(-0.5, min(0.5, rudder))  # Limit rudder authority

        # FREE mode - no sail control
        wingsail = 0.0

        return rudder, wingsail, 0.0

    def analyze(self) -> TestResult:
        """Analyze storm survival"""
        super().analyze()

        history = self.sim.get_history()
        if not history:
            return self.result

        # Heel statistics
        heels = [abs(s.heel_angle_deg) for s in history]
        avg_heel = sum(heels) / len(heels)

        # Roll rate (stability indicator)
        roll_rates = [abs(s.gyro.x) for s in history]
        avg_roll_rate = sum(roll_rates) / len(roll_rates)
        max_roll_rate = max(roll_rates)

        self.result.add_metric(
            "max_heel", self.max_heel_observed, "deg",
            threshold_max=45.0,  # Should not exceed 45 degrees
            description="Maximum heel angle"
        )

        self.result.add_metric(
            "avg_heel", avg_heel, "deg",
            threshold_max=25.0,
            description="Average heel angle"
        )

        self.result.add_metric(
            "max_roll_rate", math.degrees(max_roll_rate), "deg/s",
            threshold_max=30.0,  # Rapid rolling is dangerous
            description="Maximum roll rate"
        )

        self.result.add_metric(
            "capsize_avoided", 0.0 if self.capsize_detected else 1.0, "",
            threshold_min=1.0,  # Must not capsize
            description="Capsize prevention (1=safe, 0=capsized)"
        )

        self.result.notes.append(f"Wind speed: {self.sim.env.wind_speed_ms:.1f} m/s")
        self.result.notes.append(f"Wave height: {self.sim.env.wave_height_m:.1f} m")

        return self.result


class VariableWindTest(SailboatTestBase):
    """
    Test handling of variable wind conditions

    Simulates wind changing direction and speed, testing adaptation
    """

    def __init__(self):
        super().__init__(
            name="Variable Wind Adaptation",
            description="Tests response to changing wind conditions"
        )
        self.duration = 180.0
        self.wingsail_type = WingsailType.WINGSAIL_FLAP
        self.beaufort = BeaufortScale.MODERATE_BREEZE
        self.sea_state = SeaState.SLIGHT

        self.wind_shifts = 0
        self.last_wind_dir = 0.0

    def setup(self):
        """Setup variable wind test"""
        self.sim.init(self.wingsail_type)
        self.sim.set_beaufort(self.beaufort)
        self.sim.set_sea_state(self.sea_state)

        self.sim.env.wind_direction_deg = 0.0
        self.sim.set_heading(90.0)  # Start on beam reach

        self.last_wind_dir = 0.0
        self.wind_shifts = 0

    def control_loop(self, state: SimState, time_s: float) -> Tuple[float, float, float]:
        """
        Adaptive control with wind shifts

        Wind shifts every 30 seconds by 45 degrees
        """
        # Shift wind periodically
        shift_period = 30.0
        base_dir = (int(time_s / shift_period) * 45) % 360

        if base_dir != self.last_wind_dir:
            self.wind_shifts += 1
            self.last_wind_dir = base_dir

        self.sim.env.wind_direction_deg = float(base_dir)

        # Adapt heading to maintain beam reach
        target_heading = base_dir + 90.0
        if target_heading >= 360:
            target_heading -= 360

        current_heading = math.degrees(state.yaw_rad)
        if current_heading < 0:
            current_heading += 360.0

        heading_error = target_heading - current_heading
        if heading_error > 180:
            heading_error -= 360
        elif heading_error < -180:
            heading_error += 360

        # More aggressive steering to follow wind
        rudder = heading_error * 0.03
        rudder = max(-1.0, min(1.0, rudder))

        # Flap adjustment based on apparent wind
        app_wind_dir = state.apparent_wind_dir_deg
        if app_wind_dir > 0:
            wingsail = 0.6
        else:
            wingsail = -0.6

        return rudder, wingsail, 0.0

    def analyze(self) -> TestResult:
        """Analyze wind adaptation"""
        super().analyze()

        history = self.sim.get_history()
        if not history:
            return self.result

        # Speed consistency across wind shifts
        speeds = [s.speed_ms for s in history]
        avg_speed = sum(speeds) / len(speeds)
        min_speed = min(speeds)

        # Recovery metric - how quickly speed recovers after shift
        speed_variance = sum((s - avg_speed)**2 for s in speeds) / len(speeds)

        self.result.add_metric(
            "wind_shifts", self.wind_shifts, "shifts",
            description="Number of wind shifts handled"
        )

        self.result.add_metric(
            "avg_speed", avg_speed, "m/s",
            threshold_min=1.0,
            description="Average speed across conditions"
        )

        self.result.add_metric(
            "min_speed", min_speed, "m/s",
            threshold_min=0.3,  # Should maintain some speed
            description="Minimum speed during transitions"
        )

        self.result.add_metric(
            "speed_consistency", 1.0 / (1.0 + math.sqrt(speed_variance)), "",
            threshold_min=0.5,  # Higher = more consistent
            description="Speed consistency metric (0-1)"
        )

        return self.result


class LowWindMotorAssistTest(SailboatTestBase):
    """
    Test motor assist in low wind conditions

    Verifies motor kicks in when wind is insufficient
    """

    def __init__(self):
        super().__init__(
            name="Low Wind Motor Assist",
            description="Tests motor assist activation in light air"
        )
        self.duration = 90.0
        self.wingsail_type = WingsailType.WINGSAIL_FLAP
        self.beaufort = BeaufortScale.LIGHT_AIR
        self.sea_state = SeaState.CALM_RIPPLED

        self.motor_active_time = 0.0

    def setup(self):
        """Setup low wind test"""
        self.sim.init(self.wingsail_type)
        self.sim.set_beaufort(self.beaufort)
        self.sim.set_sea_state(self.sea_state)
        self.sim.motor_enabled = True  # Enable motor

        self.sim.env.wind_direction_deg = 0.0
        self.sim.set_heading(90.0)

        self.motor_active_time = 0.0

    def control_loop(self, state: SimState, time_s: float) -> Tuple[float, float, float]:
        """
        Motor assist control

        Use motor when sail power is insufficient
        """
        # Heading control
        target_heading = 90.0
        current_heading = math.degrees(state.yaw_rad)
        if current_heading < 0:
            current_heading += 360.0

        heading_error = target_heading - current_heading
        if heading_error > 180:
            heading_error -= 360
        elif heading_error < -180:
            heading_error += 360

        rudder = heading_error * 0.02
        rudder = max(-1.0, min(1.0, rudder))

        # Sail control
        wingsail = 0.6

        # Motor assist if speed is low
        min_speed = 0.5  # m/s
        if state.speed_ms < min_speed:
            throttle = 0.5  # Half throttle assist
            self.motor_active_time += 0.02  # dt
        else:
            throttle = 0.0

        return rudder, wingsail, throttle

    def analyze(self) -> TestResult:
        """Analyze motor assist usage"""
        super().analyze()

        history = self.sim.get_history()
        if not history:
            return self.result

        # Speed with motor assist
        speeds = [s.speed_ms for s in history]
        avg_speed = sum(speeds) / len(speeds)

        # Progress
        start_lat = history[0].lat_deg * 111320.0
        start_lon = history[0].lon_deg * 111320.0
        end_lat = history[-1].lat_deg * 111320.0
        end_lon = history[-1].lon_deg * 111320.0

        distance = math.sqrt((end_lat - start_lat)**2 + (end_lon - start_lon)**2)

        motor_ratio = self.motor_active_time / self.duration

        self.result.add_metric(
            "avg_speed", avg_speed, "m/s",
            threshold_min=0.5,  # Should maintain reasonable speed
            description="Average speed with motor assist"
        )

        self.result.add_metric(
            "distance", distance, "m",
            threshold_min=20.0,
            description="Distance traveled"
        )

        self.result.add_metric(
            "motor_usage", motor_ratio * 100, "%",
            description="Percentage of time motor was active"
        )

        self.result.notes.append(f"Wind speed: {self.sim.env.wind_speed_ms:.1f} m/s (light air)")
        self.result.notes.append(f"Motor active: {self.motor_active_time:.1f}s")

        return self.result


def run_storm_tests():
    """Run all storm stability tests"""
    from .test_runner import TestRunner

    runner = TestRunner("Storm Stability Tests")
    runner.add_test(GaleConditionsTest())
    runner.add_test(VariableWindTest())
    runner.add_test(LowWindMotorAssistTest())

    results = runner.run_all()

    runner.generate_report("reports/storm_stability.html", format="html")
    runner.generate_report("reports/storm_stability.json", format="json")

    return results


if __name__ == "__main__":
    run_storm_tests()
