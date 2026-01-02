"""
USV Sailboat SITL Test Framework

This package provides automated testing for the USV wingsail control system.
"""

from .sailboat_sim import SailboatSimulator, WingsailType, BeaufortScale, SeaState
from .test_base import SailboatTestBase, TestResult
from .test_runner import TestRunner

__version__ = "1.0.0"
__all__ = [
    "SailboatSimulator",
    "WingsailType",
    "BeaufortScale",
    "SeaState",
    "SailboatTestBase",
    "TestResult",
    "TestRunner",
]
