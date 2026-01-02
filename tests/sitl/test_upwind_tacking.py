"""
Upwind Tacking Test

Tests the sailboat's ability to sail upwind using tacking maneuvers.
Verifies:
- Tack timing and execution
- VMG optimization
- Heading accuracy after tack
- Cross-track error management
"""

import math
from typing import Tuple

from .sailboat_sim import SailboatSimulator, WingsailType, BeaufortScale, SeaState, SimState
from .test_base import SailboatTestBase, TestResult, TestStatus


class UpwindTackingTest(SailboatTestBase):
    """
    Test upwind sailing with automatic tacking

    The boat attempts to sail toward the wind source by tacking
    between port and starboard close-hauled courses.
    """

    def __init__(self):
        super().__init__(
            name="Upwind Tacking Test",
            description="Tests tacking performance when sailing toward the wind"
        )
        self.duration = 120.0  # 2 minutes
        self.wingsail_type = WingsailType.WINGSAIL_FLAP
        self.beaufort = BeaufortScale.MODERATE_BREEZE
        self.sea_state = SeaState.SLIGHT

        # Tacking parameters
        self.no_go_angle = 45.0  # degrees
        self.tack_interval = 25.0  # seconds between tacks
        self.target_heading = 0.0  # North (into the wind)

        # State
        self.current_tack = "starboard"  # or "port"
        self.last_tack_time = 0.0
        self.tack_count = 0

    def setup(self):
        """Setup upwind test scenario"""
        self.sim.init(self.wingsail_type)
        self.sim.set_beaufort(self.beaufort)
        self.sim.set_sea_state(self.sea_state)

        # Wind from North, initial heading NE (starboard tack)
        self.sim.env.wind_direction_deg = 0.0
        self.sim.set_heading(45.0)  # Close-hauled on starboard

        self.current_tack = "starboard"
        self.last_tack_time = 0.0
        self.tack_count = 0

    def control_loop(self, state: SimState, time_s: float) -> Tuple[float, float, float]:
        """
        Tacking control algorithm

        Alternates between port and starboard close-hauled courses
        """
        # Check if it's time to tack
        if time_s - self.last_tack_time > self.tack_interval:
            self.last_tack_time = time_s
            self.tack_count += 1

            if self.current_tack == "starboard":
                self.current_tack = "port"
            else:
                self.current_tack = "starboard"

        # Target heading based on current tack
        if self.current_tack == "starboard":
            target_heading = self.no_go_angle  # 45 degrees (NE)
        else:
            target_heading = 360.0 - self.no_go_angle  # 315 degrees (NW)

        # Calculate heading error
        current_heading = math.degrees(state.yaw_rad)
        if current_heading < 0:
            current_heading += 360.0

        heading_error = target_heading - current_heading
        if heading_error > 180:
            heading_error -= 360
        elif heading_error < -180:
            heading_error += 360

        # Rudder control (P controller)
        rudder_gain = 0.02
        rudder = heading_error * rudder_gain
        rudder = max(-1.0, min(1.0, rudder))

        # Wingsail control based on tack
        # Positive flap for starboard tack, negative for port
        if self.current_tack == "starboard":
            wingsail = 0.7
        else:
            wingsail = -0.7

        return rudder, wingsail, 0.0

    def analyze(self) -> TestResult:
        """Analyze upwind tacking performance"""
        # Call base analysis first
        super().analyze()

        history = self.sim.get_history()
        if not history:
            return self.result

        # Calculate northward progress (VMG toward wind)
        start_lat = history[0].lat_deg * 111320.0
        end_lat = history[-1].lat_deg * 111320.0
        northward_distance = end_lat - start_lat

        # Average VMG
        vmgs = [s.vmg for s in history]
        avg_vmg = sum(vmgs) / len(vmgs) if vmgs else 0

        # Tacking metrics
        self.result.add_metric(
            "tack_count", self.tack_count, "tacks",
            description="Number of tacks performed"
        )

        self.result.add_metric(
            "northward_progress", northward_distance, "m",
            threshold_min=10.0,  # Should make at least 10m progress
            description="Distance made good toward wind"
        )

        self.result.add_metric(
            "avg_vmg", avg_vmg, "m/s",
            threshold_min=0.5,  # At least 0.5 m/s VMG
            description="Average velocity made good"
        )

        # Heading accuracy - check time spent on correct heading
        correct_heading_time = 0
        for s in history:
            heading = math.degrees(s.yaw_rad)
            if heading < 0:
                heading += 360

            # Check if within 10 degrees of target close-hauled heading
            if abs(heading - self.no_go_angle) < 15 or abs(heading - (360 - self.no_go_angle)) < 15:
                correct_heading_time += 0.1  # record interval

        heading_accuracy = correct_heading_time / (len(history) * 0.1) * 100

        self.result.add_metric(
            "heading_accuracy", heading_accuracy, "%",
            threshold_min=60.0,  # At least 60% time on correct heading
            description="Percentage of time on close-hauled heading"
        )

        self.result.notes.append(f"Tacking interval: {self.tack_interval}s")
        self.result.notes.append(f"No-go angle: {self.no_go_angle}°")

        return self.result


class CrossTrackTackingTest(SailboatTestBase):
    """
    Test tacking triggered by cross-track error

    Instead of time-based tacking, tacks when the boat drifts
    too far from the direct line to the target.
    """

    def __init__(self):
        super().__init__(
            name="Cross-Track Tacking Test",
            description="Tests tacking based on cross-track error limits"
        )
        self.duration = 180.0  # 3 minutes
        self.wingsail_type = WingsailType.WINGSAIL_FLAP
        self.beaufort = BeaufortScale.FRESH_BREEZE
        self.sea_state = SeaState.MODERATE

        # Cross-track parameters
        self.xtrack_max = 20.0  # meters
        self.no_go_angle = 45.0

        # State
        self.current_tack = "starboard"
        self.tack_count = 0

    def setup(self):
        """Setup cross-track test"""
        self.sim.init(self.wingsail_type)
        self.sim.set_beaufort(self.beaufort)
        self.sim.set_sea_state(self.sea_state)

        self.sim.env.wind_direction_deg = 0.0
        self.sim.set_heading(45.0)

        self.current_tack = "starboard"
        self.tack_count = 0

    def control_loop(self, state: SimState, time_s: float) -> Tuple[float, float, float]:
        """
        Cross-track based tacking control

        Tacks when the boat drifts too far from the centerline
        """
        # Calculate cross-track error (east-west position)
        xtrack = state.lon_deg * 111320.0 * math.cos(math.radians(state.lat_deg))

        # Check if we need to tack based on cross-track error
        if self.current_tack == "starboard" and xtrack > self.xtrack_max:
            self.current_tack = "port"
            self.tack_count += 1
        elif self.current_tack == "port" and xtrack < -self.xtrack_max:
            self.current_tack = "starboard"
            self.tack_count += 1

        # Target heading
        if self.current_tack == "starboard":
            target_heading = self.no_go_angle
        else:
            target_heading = 360.0 - self.no_go_angle

        # Heading control
        current_heading = math.degrees(state.yaw_rad)
        if current_heading < 0:
            current_heading += 360.0

        heading_error = target_heading - current_heading
        if heading_error > 180:
            heading_error -= 360
        elif heading_error < -180:
            heading_error += 360

        rudder = heading_error * 0.025
        rudder = max(-1.0, min(1.0, rudder))

        # Wingsail
        wingsail = 0.7 if self.current_tack == "starboard" else -0.7

        return rudder, wingsail, 0.0

    def analyze(self) -> TestResult:
        """Analyze cross-track tacking performance"""
        super().analyze()

        history = self.sim.get_history()
        if not history:
            return self.result

        # Calculate cross-track statistics
        xtracks = [s.lon_deg * 111320.0 * math.cos(math.radians(s.lat_deg)) for s in history]
        max_xtrack = max(abs(x) for x in xtracks)
        avg_xtrack = sum(abs(x) for x in xtracks) / len(xtracks)

        # Northward progress
        start_lat = history[0].lat_deg * 111320.0
        end_lat = history[-1].lat_deg * 111320.0
        northward_distance = end_lat - start_lat

        self.result.add_metric(
            "tack_count", self.tack_count, "tacks",
            description="Number of tacks performed"
        )

        self.result.add_metric(
            "max_xtrack", max_xtrack, "m",
            threshold_max=self.xtrack_max * 1.2,  # Allow 20% overshoot
            description="Maximum cross-track error"
        )

        self.result.add_metric(
            "avg_xtrack", avg_xtrack, "m",
            description="Average cross-track error"
        )

        self.result.add_metric(
            "northward_progress", northward_distance, "m",
            threshold_min=20.0,
            description="Distance made good toward wind"
        )

        self.result.notes.append(f"Cross-track limit: ±{self.xtrack_max}m")

        return self.result


def run_upwind_tests():
    """Run all upwind tacking tests"""
    from .test_runner import TestRunner

    runner = TestRunner("Upwind Tacking Tests")
    runner.add_test(UpwindTackingTest())
    runner.add_test(CrossTrackTackingTest())

    results = runner.run_all()
    runner.generate_report("reports/upwind_tacking.html", format="html")
    runner.generate_report("reports/upwind_tacking.json", format="json")

    return results


if __name__ == "__main__":
    run_upwind_tests()
