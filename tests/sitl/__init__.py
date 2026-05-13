"""
USV Sailboat SITL Test Framework

This package provides automated testing for the USV wingsail control system.
"""

from .sailboat_sim import SailboatSimulator, WingsailType, BeaufortScale, SeaState
from .test_base import SailboatTestBase, TestResult
from .test_runner import TestRunner

# Optional visualization (requires matplotlib)
try:
    from .visualizer import (
        SailboatVisualizer,
        RealtimeVisualizer,
        animate_test,
        plot_test_results,
        plot_wingsail_comparison,
    )
    HAS_VISUALIZER = True
except ImportError:
    HAS_VISUALIZER = False

__version__ = "1.0.0"
__all__ = [
    "SailboatSimulator",
    "WingsailType",
    "BeaufortScale",
    "SeaState",
    "SailboatTestBase",
    "TestResult",
    "TestRunner",
    "HAS_VISUALIZER",
]

if HAS_VISUALIZER:
    __all__.extend([
        "SailboatVisualizer",
        "RealtimeVisualizer",
        "animate_test",
        "plot_test_results",
        "plot_wingsail_comparison",
    ])
