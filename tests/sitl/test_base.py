"""
Test Base Class for USV Sailboat Tests

Provides common functionality for all sailboat tests.
"""

import time
from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional, Callable
from enum import Enum
from abc import ABC, abstractmethod

from .sailboat_sim import SailboatSimulator, WingsailType, BeaufortScale, SeaState, SimState


class TestStatus(Enum):
    """Test result status"""
    PENDING = "pending"
    RUNNING = "running"
    PASSED = "passed"
    FAILED = "failed"
    SKIPPED = "skipped"
    ERROR = "error"


@dataclass
class TestMetric:
    """A single test metric"""
    name: str
    value: float
    unit: str
    threshold_min: Optional[float] = None
    threshold_max: Optional[float] = None
    passed: bool = True
    description: str = ""

    def check(self) -> bool:
        """Check if metric is within thresholds"""
        if self.threshold_min is not None and self.value < self.threshold_min:
            self.passed = False
        if self.threshold_max is not None and self.value > self.threshold_max:
            self.passed = False
        return self.passed


@dataclass
class TestResult:
    """Test result with metrics and status"""
    test_name: str
    status: TestStatus = TestStatus.PENDING
    duration_s: float = 0.0
    metrics: List[TestMetric] = field(default_factory=list)
    history: List[SimState] = field(default_factory=list)
    error_message: str = ""
    notes: List[str] = field(default_factory=list)

    def add_metric(self, name: str, value: float, unit: str,
                   threshold_min: Optional[float] = None,
                   threshold_max: Optional[float] = None,
                   description: str = "") -> TestMetric:
        """Add a test metric"""
        metric = TestMetric(
            name=name,
            value=value,
            unit=unit,
            threshold_min=threshold_min,
            threshold_max=threshold_max,
            description=description
        )
        metric.check()
        self.metrics.append(metric)
        return metric

    def all_metrics_passed(self) -> bool:
        """Check if all metrics passed"""
        return all(m.passed for m in self.metrics)

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization"""
        return {
            "test_name": self.test_name,
            "status": self.status.value,
            "duration_s": self.duration_s,
            "metrics": [
                {
                    "name": m.name,
                    "value": m.value,
                    "unit": m.unit,
                    "threshold_min": m.threshold_min,
                    "threshold_max": m.threshold_max,
                    "passed": m.passed,
                    "description": m.description
                }
                for m in self.metrics
            ],
            "error_message": self.error_message,
            "notes": self.notes
        }


class SailboatTestBase(ABC):
    """
    Base class for sailboat tests

    Subclass this to create specific test scenarios.
    """

    def __init__(self, name: str, description: str = ""):
        self.name = name
        self.description = description
        self.sim = SailboatSimulator()
        self.result = TestResult(test_name=name)

        # Default test parameters
        self.duration = 60.0  # seconds
        self.dt = 0.02  # 50 Hz
        self.wingsail_type = WingsailType.WINGSAIL_FLAP
        self.beaufort = BeaufortScale.MODERATE_BREEZE
        self.sea_state = SeaState.SLIGHT

    def setup(self):
        """Setup test environment - override in subclass"""
        self.sim.init(self.wingsail_type)
        self.sim.set_beaufort(self.beaufort)
        self.sim.set_sea_state(self.sea_state)

    @abstractmethod
    def control_loop(self, state: SimState, time_s: float) -> tuple:
        """
        Control loop - must be implemented by subclass

        Args:
            state: Current simulation state
            time_s: Current simulation time

        Returns:
            Tuple of (rudder_input, wingsail_input, throttle_input)
        """
        pass

    def analyze(self) -> TestResult:
        """
        Analyze test results - override in subclass to add custom metrics

        Returns:
            TestResult with metrics
        """
        history = self.sim.get_history()
        if not history:
            self.result.status = TestStatus.ERROR
            self.result.error_message = "No history data recorded"
            return self.result

        # Basic metrics
        speeds = [s.speed_ms for s in history]
        heels = [abs(s.heel_angle_deg) for s in history]

        self.result.add_metric(
            "avg_speed", sum(speeds) / len(speeds), "m/s",
            description="Average boat speed"
        )
        self.result.add_metric(
            "max_speed", max(speeds), "m/s",
            description="Maximum boat speed"
        )
        self.result.add_metric(
            "max_heel", max(heels), "deg",
            threshold_max=45.0,  # Capsize threshold
            description="Maximum heel angle"
        )

        # VMG if available
        vmgs = [s.vmg for s in history if s.vmg != 0]
        if vmgs:
            self.result.add_metric(
                "avg_vmg", sum(vmgs) / len(vmgs), "m/s",
                description="Average VMG toward wind"
            )

        self.result.history = history
        return self.result

    def run(self) -> TestResult:
        """
        Run the complete test

        Returns:
            TestResult with all metrics and status
        """
        self.result.status = TestStatus.RUNNING
        start_time = time.time()

        try:
            # Setup
            self.setup()

            # Run simulation
            steps = int(self.duration / self.dt)
            for step in range(steps):
                t = step * self.dt
                state = self.sim.get_state()

                # Get control inputs from subclass
                rudder, wingsail, throttle = self.control_loop(state, t)

                # Apply controls
                self.sim.set_rudder(rudder)
                self.sim.set_wingsail(wingsail)
                self.sim.set_throttle(throttle)

                # Update simulation
                self.sim.update(self.dt)

            # Analyze results
            self.analyze()

            # Determine pass/fail
            if self.result.all_metrics_passed():
                self.result.status = TestStatus.PASSED
            else:
                self.result.status = TestStatus.FAILED

        except Exception as e:
            self.result.status = TestStatus.ERROR
            self.result.error_message = str(e)

        self.result.duration_s = time.time() - start_time
        return self.result

    def print_result(self):
        """Print test result summary"""
        status_icons = {
            TestStatus.PASSED: "✅",
            TestStatus.FAILED: "❌",
            TestStatus.ERROR: "⚠️",
            TestStatus.SKIPPED: "⏭️",
            TestStatus.PENDING: "⏳",
            TestStatus.RUNNING: "🔄"
        }

        icon = status_icons.get(self.result.status, "?")
        print(f"\n{icon} {self.name}: {self.result.status.value.upper()}")
        print(f"   Duration: {self.result.duration_s:.2f}s")

        if self.result.error_message:
            print(f"   Error: {self.result.error_message}")

        print("   Metrics:")
        for m in self.result.metrics:
            status = "✓" if m.passed else "✗"
            threshold_str = ""
            if m.threshold_min is not None:
                threshold_str += f" (min: {m.threshold_min})"
            if m.threshold_max is not None:
                threshold_str += f" (max: {m.threshold_max})"
            print(f"     {status} {m.name}: {m.value:.3f} {m.unit}{threshold_str}")

        for note in self.result.notes:
            print(f"   Note: {note}")
