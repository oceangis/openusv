#!/usr/bin/env python3
"""
USV Sailboat Test Suite - Main Entry Point

Run all tests and generate comprehensive reports.

Usage:
    python run_all_tests.py              # Run all tests
    python run_all_tests.py --quick      # Quick test subset
    python run_all_tests.py --wingsail   # Wingsail comparison only
    python run_all_tests.py --storm      # Storm tests only
    python run_all_tests.py --upwind     # Upwind tests only
"""

import sys
import os
import argparse
from datetime import datetime
from pathlib import Path

# Add sitl package to path
sys.path.insert(0, str(Path(__file__).parent))

from sitl import TestRunner, WingsailType
from sitl.test_upwind_tacking import UpwindTackingTest, CrossTrackTackingTest
from sitl.test_wingsail_modes import (
    BeamReachComparisonTest,
    UpwindComparisonTest,
    DownwindComparisonTest
)
from sitl.test_storm_stability import (
    GaleConditionsTest,
    VariableWindTest,
    LowWindMotorAssistTest
)


def create_reports_dir():
    """Create reports directory if it doesn't exist"""
    reports_dir = Path(__file__).parent / "reports"
    reports_dir.mkdir(exist_ok=True)
    return reports_dir


def run_all_tests(verbose: bool = True) -> int:
    """
    Run complete test suite

    Returns:
        Exit code (0 = all passed, 1 = some failed)
    """
    reports_dir = create_reports_dir()

    runner = TestRunner("USV Sailboat Complete Test Suite")

    # Upwind tacking tests
    runner.add_test(UpwindTackingTest())
    runner.add_test(CrossTrackTackingTest())

    # Wingsail mode comparison
    for wingsail_type in WingsailType:
        runner.add_test(BeamReachComparisonTest(wingsail_type))
        runner.add_test(UpwindComparisonTest(wingsail_type))
        runner.add_test(DownwindComparisonTest(wingsail_type))

    # Storm stability tests
    runner.add_test(GaleConditionsTest())
    runner.add_test(VariableWindTest())
    runner.add_test(LowWindMotorAssistTest())

    # Run tests
    results = runner.run_all(verbose=verbose)

    # Generate reports
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    runner.generate_report(reports_dir / f"full_report_{timestamp}.html", format="html")
    runner.generate_report(reports_dir / f"full_report_{timestamp}.json", format="json")
    runner.generate_report(reports_dir / f"full_report_{timestamp}.md", format="markdown")

    # Also generate latest symlink
    runner.generate_report(reports_dir / "latest.html", format="html")
    runner.generate_report(reports_dir / "latest.json", format="json")

    # Return exit code
    summary = runner.get_summary()
    return 0 if summary["failed"] == 0 and summary["errors"] == 0 else 1


def run_quick_tests(verbose: bool = True) -> int:
    """Run quick subset of tests"""
    reports_dir = create_reports_dir()

    runner = TestRunner("USV Sailboat Quick Tests")

    # Just one of each type
    runner.add_test(UpwindTackingTest())
    runner.add_test(BeamReachComparisonTest(WingsailType.WINGSAIL_FLAP))
    runner.add_test(GaleConditionsTest())

    results = runner.run_all(verbose=verbose)
    runner.generate_report(reports_dir / "quick_test.html", format="html")

    summary = runner.get_summary()
    return 0 if summary["failed"] == 0 and summary["errors"] == 0 else 1


def run_wingsail_tests(verbose: bool = True) -> int:
    """Run wingsail comparison tests only"""
    reports_dir = create_reports_dir()

    runner = TestRunner("Wingsail Mode Comparison")

    for wingsail_type in WingsailType:
        runner.add_test(BeamReachComparisonTest(wingsail_type))
        runner.add_test(UpwindComparisonTest(wingsail_type))
        runner.add_test(DownwindComparisonTest(wingsail_type))

    results = runner.run_all(verbose=verbose)
    runner.generate_report(reports_dir / "wingsail_comparison.html", format="html")
    runner.generate_report(reports_dir / "wingsail_comparison.json", format="json")

    # Print comparison table
    print("\n" + "=" * 70)
    print("  PERFORMANCE COMPARISON TABLE")
    print("=" * 70)
    print(f"{'Test':<30} {'ROTATION':>12} {'FLAP':>12} {'FREE':>12}")
    print("-" * 70)

    conditions = ["Beam Reach", "Upwind VMG", "Downwind"]
    for condition in conditions:
        row = f"{condition:<30}"
        for wt in WingsailType:
            result = next((r for r in results if condition in r.test_name and wt.name in r.test_name), None)
            if result:
                speed = next((m.value for m in result.metrics if "speed" in m.name.lower() or "vmg" in m.name.lower()), 0)
                row += f"{speed:>10.2f} ms"
            else:
                row += f"{'N/A':>12}"
        print(row)
    print("=" * 70)

    summary = runner.get_summary()
    return 0 if summary["failed"] == 0 and summary["errors"] == 0 else 1


def run_storm_tests(verbose: bool = True) -> int:
    """Run storm stability tests only"""
    reports_dir = create_reports_dir()

    runner = TestRunner("Storm Stability Tests")

    runner.add_test(GaleConditionsTest())
    runner.add_test(VariableWindTest())
    runner.add_test(LowWindMotorAssistTest())

    results = runner.run_all(verbose=verbose)
    runner.generate_report(reports_dir / "storm_stability.html", format="html")

    summary = runner.get_summary()
    return 0 if summary["failed"] == 0 and summary["errors"] == 0 else 1


def run_upwind_tests(verbose: bool = True) -> int:
    """Run upwind tacking tests only"""
    reports_dir = create_reports_dir()

    runner = TestRunner("Upwind Tacking Tests")

    runner.add_test(UpwindTackingTest())
    runner.add_test(CrossTrackTackingTest())

    results = runner.run_all(verbose=verbose)
    runner.generate_report(reports_dir / "upwind_tacking.html", format="html")

    summary = runner.get_summary()
    return 0 if summary["failed"] == 0 and summary["errors"] == 0 else 1


def main():
    parser = argparse.ArgumentParser(
        description="USV Sailboat Test Suite",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python run_all_tests.py              Run all tests
  python run_all_tests.py --quick      Quick test subset
  python run_all_tests.py --wingsail   Wingsail comparison only
  python run_all_tests.py --storm      Storm stability tests
  python run_all_tests.py --upwind     Upwind tacking tests
  python run_all_tests.py -q           Quiet mode (less output)
        """
    )

    parser.add_argument("--quick", action="store_true", help="Run quick test subset")
    parser.add_argument("--wingsail", action="store_true", help="Run wingsail comparison tests")
    parser.add_argument("--storm", action="store_true", help="Run storm stability tests")
    parser.add_argument("--upwind", action="store_true", help="Run upwind tacking tests")
    parser.add_argument("-q", "--quiet", action="store_true", help="Quiet mode")

    args = parser.parse_args()
    verbose = not args.quiet

    if args.quick:
        return run_quick_tests(verbose)
    elif args.wingsail:
        return run_wingsail_tests(verbose)
    elif args.storm:
        return run_storm_tests(verbose)
    elif args.upwind:
        return run_upwind_tests(verbose)
    else:
        return run_all_tests(verbose)


if __name__ == "__main__":
    sys.exit(main())
