"""
Wingsail Mode Comparison Tests

Compares the performance of three wingsail control types:
- WINGSAIL_ROTATION: Direct angle control
- WINGSAIL_FLAP: Flap-controlled, self-aligning
- WINGSAIL_FREE: Self-balancing, rudder-only control
"""

import math
from typing import Tuple, List, Dict

from .sailboat_sim import SailboatSimulator, WingsailType, BeaufortScale, SeaState, SimState
from .test_base import SailboatTestBase, TestResult, TestStatus


class BeamReachComparisonTest(SailboatTestBase):
    """
    Beam reach comparison across all wingsail modes

    Beam reach (wind perpendicular to course) is typically the
    fastest point of sail and good for comparing raw performance.
    """

    def __init__(self, wingsail_type: WingsailType):
        type_names = {
            WingsailType.WINGSAIL_ROTATION: "ROTATION",
            WingsailType.WINGSAIL_FLAP: "FLAP",
            WingsailType.WINGSAIL_FREE: "FREE"
        }

        super().__init__(
            name=f"Beam Reach - {type_names[wingsail_type]}",
            description=f"Tests {type_names[wingsail_type]} mode on beam reach"
        )
        self.duration = 60.0
        self.wingsail_type = wingsail_type
        self.beaufort = BeaufortScale.FRESH_BREEZE
        self.sea_state = SeaState.SLIGHT

    def setup(self):
        """Setup beam reach test"""
        self.sim.init(self.wingsail_type)
        self.sim.set_beaufort(self.beaufort)
        self.sim.set_sea_state(self.sea_state)

        # Wind from North, heading East (beam reach)
        self.sim.env.wind_direction_deg = 0.0
        self.sim.set_heading(90.0)

    def control_loop(self, state: SimState, time_s: float) -> Tuple[float, float, float]:
        """
        Simple beam reach control

        Maintain heading with constant sail setting
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

        # Wingsail control depends on mode
        if self.wingsail_type == WingsailType.WINGSAIL_ROTATION:
            wingsail = 0.6  # ~24 degrees
        elif self.wingsail_type == WingsailType.WINGSAIL_FLAP:
            wingsail = 0.625  # Normal flap
        else:  # FREE
            wingsail = 0.0  # No control in FREE mode

        return rudder, wingsail, 0.0

    def analyze(self) -> TestResult:
        """Analyze beam reach performance"""
        super().analyze()

        history = self.sim.get_history()
        if not history:
            return self.result

        # Steady-state speed (last 20 seconds)
        steady_state_history = history[-200:] if len(history) > 200 else history
        speeds = [s.speed_ms for s in steady_state_history]

        steady_speed = sum(speeds) / len(speeds)
        speed_variance = sum((s - steady_speed)**2 for s in speeds) / len(speeds)

        self.result.add_metric(
            "steady_speed", steady_speed, "m/s",
            threshold_min=1.0,  # Should reach at least 1 m/s
            description="Steady-state speed"
        )

        self.result.add_metric(
            "speed_stability", math.sqrt(speed_variance), "m/s",
            threshold_max=0.5,  # Low variance = stable
            description="Speed standard deviation"
        )

        # Distance traveled
        distance = 0.0
        for i in range(1, len(history)):
            dx = (history[i].lat_deg - history[i-1].lat_deg) * 111320.0
            dy = (history[i].lon_deg - history[i-1].lon_deg) * 111320.0 * math.cos(math.radians(history[i].lat_deg))
            distance += math.sqrt(dx**2 + dy**2)

        self.result.add_metric(
            "distance", distance, "m",
            description="Total distance traveled"
        )

        return self.result


class UpwindComparisonTest(SailboatTestBase):
    """
    Upwind (close-hauled) comparison across wingsail modes

    Tests VMG performance when sailing close to the wind
    """

    def __init__(self, wingsail_type: WingsailType):
        type_names = {
            WingsailType.WINGSAIL_ROTATION: "ROTATION",
            WingsailType.WINGSAIL_FLAP: "FLAP",
            WingsailType.WINGSAIL_FREE: "FREE"
        }

        super().__init__(
            name=f"Upwind VMG - {type_names[wingsail_type]}",
            description=f"Tests {type_names[wingsail_type]} mode sailing upwind"
        )
        self.duration = 90.0
        self.wingsail_type = wingsail_type
        self.beaufort = BeaufortScale.MODERATE_BREEZE
        self.sea_state = SeaState.SLIGHT

        self.no_go_angle = 45.0

    def setup(self):
        """Setup upwind test"""
        self.sim.init(self.wingsail_type)
        self.sim.set_beaufort(self.beaufort)
        self.sim.set_sea_state(self.sea_state)

        # Wind from North, heading NE (close-hauled)
        self.sim.env.wind_direction_deg = 0.0
        self.sim.set_heading(self.no_go_angle)

    def control_loop(self, state: SimState, time_s: float) -> Tuple[float, float, float]:
        """
        Close-hauled sailing control
        """
        # Maintain close-hauled heading
        target_heading = self.no_go_angle
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

        # Wingsail - tighter settings for upwind
        if self.wingsail_type == WingsailType.WINGSAIL_ROTATION:
            wingsail = 0.5  # Tighter angle
        elif self.wingsail_type == WingsailType.WINGSAIL_FLAP:
            wingsail = 0.75  # More flap for upwind power
        else:  # FREE
            wingsail = 0.0

        return rudder, wingsail, 0.0

    def analyze(self) -> TestResult:
        """Analyze upwind VMG performance"""
        super().analyze()

        history = self.sim.get_history()
        if not history:
            return self.result

        # VMG toward wind
        vmgs = [s.vmg for s in history]
        avg_vmg = sum(vmgs) / len(vmgs)
        max_vmg = max(vmgs)

        # Northward progress
        start_lat = history[0].lat_deg * 111320.0
        end_lat = history[-1].lat_deg * 111320.0
        northward_progress = end_lat - start_lat

        self.result.add_metric(
            "avg_vmg", avg_vmg, "m/s",
            threshold_min=0.3 if self.wingsail_type != WingsailType.WINGSAIL_FREE else 0.1,
            description="Average VMG toward wind"
        )

        self.result.add_metric(
            "max_vmg", max_vmg, "m/s",
            description="Maximum VMG achieved"
        )

        self.result.add_metric(
            "northward_progress", northward_progress, "m",
            threshold_min=5.0 if self.wingsail_type != WingsailType.WINGSAIL_FREE else 1.0,
            description="Distance made toward wind"
        )

        return self.result


class DownwindComparisonTest(SailboatTestBase):
    """
    Downwind (running) comparison across wingsail modes
    """

    def __init__(self, wingsail_type: WingsailType):
        type_names = {
            WingsailType.WINGSAIL_ROTATION: "ROTATION",
            WingsailType.WINGSAIL_FLAP: "FLAP",
            WingsailType.WINGSAIL_FREE: "FREE"
        }

        super().__init__(
            name=f"Downwind - {type_names[wingsail_type]}",
            description=f"Tests {type_names[wingsail_type]} mode running downwind"
        )
        self.duration = 60.0
        self.wingsail_type = wingsail_type
        self.beaufort = BeaufortScale.FRESH_BREEZE
        self.sea_state = SeaState.MODERATE

    def setup(self):
        """Setup downwind test"""
        self.sim.init(self.wingsail_type)
        self.sim.set_beaufort(self.beaufort)
        self.sim.set_sea_state(self.sea_state)

        # Wind from North, heading South (running)
        self.sim.env.wind_direction_deg = 0.0
        self.sim.set_heading(180.0)

    def control_loop(self, state: SimState, time_s: float) -> Tuple[float, float, float]:
        """
        Downwind sailing control
        """
        target_heading = 180.0
        current_heading = math.degrees(state.yaw_rad)
        if current_heading < 0:
            current_heading += 360.0

        heading_error = target_heading - current_heading
        if heading_error > 180:
            heading_error -= 360
        elif heading_error < -180:
            heading_error += 360

        rudder = heading_error * 0.015  # Less aggressive downwind
        rudder = max(-1.0, min(1.0, rudder))

        # Wingsail - max angle for downwind
        if self.wingsail_type == WingsailType.WINGSAIL_ROTATION:
            wingsail = 1.0  # Full angle
        elif self.wingsail_type == WingsailType.WINGSAIL_FLAP:
            wingsail = 0.5  # Moderate flap
        else:
            wingsail = 0.0

        return rudder, wingsail, 0.0

    def analyze(self) -> TestResult:
        """Analyze downwind performance"""
        super().analyze()

        history = self.sim.get_history()
        if not history:
            return self.result

        # Southward progress
        start_lat = history[0].lat_deg * 111320.0
        end_lat = history[-1].lat_deg * 111320.0
        southward_progress = start_lat - end_lat  # Negative lat = south

        speeds = [s.speed_ms for s in history[-100:]]  # Last 10 seconds
        steady_speed = sum(speeds) / len(speeds)

        self.result.add_metric(
            "steady_speed", steady_speed, "m/s",
            threshold_min=1.5,
            description="Steady-state downwind speed"
        )

        self.result.add_metric(
            "southward_progress", southward_progress, "m",
            threshold_min=50.0,
            description="Distance sailed downwind"
        )

        return self.result


def run_wingsail_comparison():
    """Run complete wingsail mode comparison"""
    from .test_runner import TestRunner

    runner = TestRunner("Wingsail Mode Comparison")

    # Add tests for each mode and condition
    for wingsail_type in WingsailType:
        runner.add_test(BeamReachComparisonTest(wingsail_type))
        runner.add_test(UpwindComparisonTest(wingsail_type))
        runner.add_test(DownwindComparisonTest(wingsail_type))

    results = runner.run_all()

    # Generate reports
    runner.generate_report("reports/wingsail_comparison.html", format="html")
    runner.generate_report("reports/wingsail_comparison.json", format="json")
    runner.generate_report("reports/wingsail_comparison.md", format="markdown")

    # Print comparison summary
    print("\n" + "=" * 60)
    print("  WINGSAIL MODE COMPARISON SUMMARY")
    print("=" * 60)

    # Group results by point of sail
    conditions = ["Beam Reach", "Upwind VMG", "Downwind"]
    for condition in conditions:
        print(f"\n{condition}:")
        condition_results = [r for r in results if condition in r.test_name]
        for r in condition_results:
            mode = r.test_name.split(" - ")[1]
            speed_metric = next((m for m in r.metrics if "speed" in m.name.lower() or "vmg" in m.name.lower()), None)
            if speed_metric:
                print(f"  {mode:12s}: {speed_metric.value:.2f} {speed_metric.unit}")

    return results


if __name__ == "__main__":
    run_wingsail_comparison()
