"""
Test Runner for USV Sailboat Tests

Manages test execution and report generation.
"""

import json
import time
from datetime import datetime
from typing import List, Dict, Any, Optional
from pathlib import Path

from .test_base import SailboatTestBase, TestResult, TestStatus


class TestRunner:
    """
    Test runner that executes multiple tests and generates reports
    """

    def __init__(self, name: str = "USV Sailboat Test Suite"):
        self.name = name
        self.tests: List[SailboatTestBase] = []
        self.results: List[TestResult] = []
        self.start_time: Optional[datetime] = None
        self.end_time: Optional[datetime] = None

    def add_test(self, test: SailboatTestBase):
        """Add a test to the suite"""
        self.tests.append(test)

    def add_tests(self, tests: List[SailboatTestBase]):
        """Add multiple tests to the suite"""
        self.tests.extend(tests)

    def run_all(self, verbose: bool = True) -> List[TestResult]:
        """
        Run all tests in the suite

        Args:
            verbose: Print progress and results

        Returns:
            List of TestResult objects
        """
        self.start_time = datetime.now()
        self.results = []

        if verbose:
            print("=" * 60)
            print(f"  {self.name}")
            print(f"  Started: {self.start_time.strftime('%Y-%m-%d %H:%M:%S')}")
            print(f"  Tests: {len(self.tests)}")
            print("=" * 60)

        for i, test in enumerate(self.tests, 1):
            if verbose:
                print(f"\n[{i}/{len(self.tests)}] Running: {test.name}")
                if test.description:
                    print(f"    {test.description}")

            result = test.run()
            self.results.append(result)

            if verbose:
                test.print_result()

        self.end_time = datetime.now()

        if verbose:
            self._print_summary()

        return self.results

    def _print_summary(self):
        """Print test suite summary"""
        passed = sum(1 for r in self.results if r.status == TestStatus.PASSED)
        failed = sum(1 for r in self.results if r.status == TestStatus.FAILED)
        errors = sum(1 for r in self.results if r.status == TestStatus.ERROR)
        skipped = sum(1 for r in self.results if r.status == TestStatus.SKIPPED)

        total_time = (self.end_time - self.start_time).total_seconds()

        print("\n" + "=" * 60)
        print("  TEST SUMMARY")
        print("=" * 60)
        print(f"  Total:   {len(self.results)}")
        print(f"  Passed:  {passed} ✅")
        print(f"  Failed:  {failed} ❌")
        print(f"  Errors:  {errors} ⚠️")
        print(f"  Skipped: {skipped} ⏭️")
        print(f"  Time:    {total_time:.2f}s")
        print("=" * 60)

        if failed + errors == 0:
            print("  🎉 ALL TESTS PASSED!")
        else:
            print("  ⚠️  SOME TESTS FAILED")
        print("=" * 60)

    def get_summary(self) -> Dict[str, Any]:
        """Get test summary as dictionary"""
        passed = sum(1 for r in self.results if r.status == TestStatus.PASSED)
        failed = sum(1 for r in self.results if r.status == TestStatus.FAILED)
        errors = sum(1 for r in self.results if r.status == TestStatus.ERROR)
        skipped = sum(1 for r in self.results if r.status == TestStatus.SKIPPED)

        return {
            "suite_name": self.name,
            "start_time": self.start_time.isoformat() if self.start_time else None,
            "end_time": self.end_time.isoformat() if self.end_time else None,
            "duration_s": (self.end_time - self.start_time).total_seconds() if self.end_time else 0,
            "total_tests": len(self.results),
            "passed": passed,
            "failed": failed,
            "errors": errors,
            "skipped": skipped,
            "success_rate": passed / len(self.results) * 100 if self.results else 0
        }

    def generate_report(self, output_path: str, format: str = "json") -> str:
        """
        Generate test report

        Args:
            output_path: Path to output file
            format: Report format ('json', 'markdown', 'html')

        Returns:
            Path to generated report
        """
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        if format == "json":
            return self._generate_json_report(output_path)
        elif format == "markdown":
            return self._generate_markdown_report(output_path)
        elif format == "html":
            return self._generate_html_report(output_path)
        else:
            raise ValueError(f"Unknown report format: {format}")

    def _generate_json_report(self, output_path: Path) -> str:
        """Generate JSON report"""
        report = {
            "summary": self.get_summary(),
            "tests": [r.to_dict() for r in self.results]
        }

        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(report, f, indent=2, ensure_ascii=False)

        return str(output_path)

    def _generate_markdown_report(self, output_path: Path) -> str:
        """Generate Markdown report"""
        summary = self.get_summary()

        lines = [
            f"# {self.name} - Test Report",
            "",
            f"**Date:** {summary['start_time'][:10] if summary['start_time'] else 'N/A'}",
            f"**Duration:** {summary['duration_s']:.2f}s",
            "",
            "## Summary",
            "",
            "| Metric | Value |",
            "|--------|-------|",
            f"| Total Tests | {summary['total_tests']} |",
            f"| Passed | {summary['passed']} ✅ |",
            f"| Failed | {summary['failed']} ❌ |",
            f"| Errors | {summary['errors']} ⚠️ |",
            f"| Success Rate | {summary['success_rate']:.1f}% |",
            "",
            "## Test Results",
            ""
        ]

        for result in self.results:
            status_icon = {
                TestStatus.PASSED: "✅",
                TestStatus.FAILED: "❌",
                TestStatus.ERROR: "⚠️",
                TestStatus.SKIPPED: "⏭️"
            }.get(result.status, "?")

            lines.extend([
                f"### {status_icon} {result.test_name}",
                "",
                f"**Status:** {result.status.value}",
                f"**Duration:** {result.duration_s:.2f}s",
                ""
            ])

            if result.error_message:
                lines.extend([
                    "**Error:**",
                    f"```",
                    result.error_message,
                    "```",
                    ""
                ])

            if result.metrics:
                lines.extend([
                    "**Metrics:**",
                    "",
                    "| Metric | Value | Unit | Status |",
                    "|--------|-------|------|--------|"
                ])
                for m in result.metrics:
                    status = "✓" if m.passed else "✗"
                    lines.append(f"| {m.name} | {m.value:.3f} | {m.unit} | {status} |")
                lines.append("")

            if result.notes:
                lines.extend([
                    "**Notes:**",
                    ""
                ])
                for note in result.notes:
                    lines.append(f"- {note}")
                lines.append("")

        with open(output_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines))

        return str(output_path)

    def _generate_html_report(self, output_path: Path) -> str:
        """Generate HTML report"""
        summary = self.get_summary()

        html = f"""<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>{self.name} - Test Report</title>
    <style>
        body {{ font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; margin: 40px; background: #f5f5f5; }}
        .container {{ max-width: 1000px; margin: 0 auto; background: white; padding: 30px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }}
        h1 {{ color: #333; border-bottom: 2px solid #007bff; padding-bottom: 10px; }}
        h2 {{ color: #555; margin-top: 30px; }}
        h3 {{ color: #666; }}
        .summary {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 20px; margin: 20px 0; }}
        .summary-card {{ background: #f8f9fa; padding: 20px; border-radius: 8px; text-align: center; }}
        .summary-card .value {{ font-size: 2em; font-weight: bold; color: #007bff; }}
        .summary-card .label {{ color: #666; margin-top: 5px; }}
        .passed {{ color: #28a745; }}
        .failed {{ color: #dc3545; }}
        .error {{ color: #ffc107; }}
        table {{ width: 100%; border-collapse: collapse; margin: 15px 0; }}
        th, td {{ padding: 10px; text-align: left; border-bottom: 1px solid #ddd; }}
        th {{ background: #f8f9fa; }}
        .test-card {{ background: #f8f9fa; padding: 20px; border-radius: 8px; margin: 15px 0; }}
        .test-card.passed {{ border-left: 4px solid #28a745; }}
        .test-card.failed {{ border-left: 4px solid #dc3545; }}
        .test-card.error {{ border-left: 4px solid #ffc107; }}
        .metric-passed {{ color: #28a745; }}
        .metric-failed {{ color: #dc3545; }}
    </style>
</head>
<body>
    <div class="container">
        <h1>{self.name}</h1>
        <p><strong>Date:</strong> {summary['start_time'][:10] if summary['start_time'] else 'N/A'} |
           <strong>Duration:</strong> {summary['duration_s']:.2f}s</p>

        <h2>Summary</h2>
        <div class="summary">
            <div class="summary-card">
                <div class="value">{summary['total_tests']}</div>
                <div class="label">Total Tests</div>
            </div>
            <div class="summary-card">
                <div class="value passed">{summary['passed']}</div>
                <div class="label">Passed</div>
            </div>
            <div class="summary-card">
                <div class="value failed">{summary['failed']}</div>
                <div class="label">Failed</div>
            </div>
            <div class="summary-card">
                <div class="value">{summary['success_rate']:.1f}%</div>
                <div class="label">Success Rate</div>
            </div>
        </div>

        <h2>Test Results</h2>
"""

        for result in self.results:
            status_class = result.status.value
            status_icon = {"passed": "✅", "failed": "❌", "error": "⚠️"}.get(status_class, "?")

            html += f"""
        <div class="test-card {status_class}">
            <h3>{status_icon} {result.test_name}</h3>
            <p><strong>Status:</strong> {result.status.value} |
               <strong>Duration:</strong> {result.duration_s:.2f}s</p>
"""

            if result.error_message:
                html += f"""
            <p><strong>Error:</strong> {result.error_message}</p>
"""

            if result.metrics:
                html += """
            <table>
                <tr><th>Metric</th><th>Value</th><th>Unit</th><th>Status</th></tr>
"""
                for m in result.metrics:
                    status_class = "metric-passed" if m.passed else "metric-failed"
                    status_symbol = "✓" if m.passed else "✗"
                    html += f"""
                <tr>
                    <td>{m.name}</td>
                    <td>{m.value:.3f}</td>
                    <td>{m.unit}</td>
                    <td class="{status_class}">{status_symbol}</td>
                </tr>
"""
                html += """
            </table>
"""

            html += """
        </div>
"""

        html += """
    </div>
</body>
</html>
"""

        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(html)

        return str(output_path)
