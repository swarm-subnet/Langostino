#!/usr/bin/env python3
"""
Automated Test Suite for AI Adapter Simulator
Runs 100 tests with varying goal positions and generates a performance report.

Usage:
    python3 run_simulator_tests.py [--max-time SECONDS] [--tests NUM]
"""

import subprocess
import time
import re
import random
import json
import argparse
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Tuple, Optional


class SimulatorTest:
    """Manages individual simulator test execution."""

    def __init__(self, test_id: int, goal_position: Tuple[float, float, float], max_time: float = 120.0):
        self.test_id = test_id
        self.goal_position = goal_position
        self.max_time = max_time
        self.start_time = None
        self.end_time = None
        self.success = False
        self.iterations = 0
        self.distance_traveled = 0.0
        self.final_distance_to_goal = None
        self.path_efficiency = None
        self.timeout = False
        self.error_message = None

    def run(self) -> Dict:
        """Execute the test and collect results."""
        print(f"\n{'='*80}")
        print(f"Test #{self.test_id:03d} - Goal: ({self.goal_position[0]:.1f}, {self.goal_position[1]:.1f}, {self.goal_position[2]:.1f})")
        print(f"{'='*80}")

        goal_str = f"[{self.goal_position[0]}, {self.goal_position[1]}, {self.goal_position[2]}]"
        cmd = [
            "ros2", "run", "swarm_ai_integration", "ai_adapter_simulator_node.py",
            "--ros-args",
            "-p", f"goal_relative_enu:={goal_str}"
        ]

        self.start_time = time.time()

        try:
            # Run the simulator process
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1
            )

            output_lines = []

            while True:
                # Check timeout
                elapsed = time.time() - self.start_time
                if elapsed > self.max_time:
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                    self.timeout = True
                    self.end_time = time.time()
                    break

                # Read output line
                line = process.stdout.readline()
                if not line and process.poll() is not None:
                    break

                if line:
                    output_lines.append(line)

                    # Parse iteration info
                    if "Iter" in line:
                        match = re.search(r'Iter (\d+).*→Goal=\s*([\d.]+)m.*Traveled=\s*([\d.]+)m', line)
                        if match:
                            self.iterations = int(match.group(1))
                            self.final_distance_to_goal = float(match.group(2))
                            self.distance_traveled = float(match.group(3))

                    # Check for success
                    if "GOAL REACHED" in line or "🎯" in line:
                        self.success = True

                    # Parse efficiency
                    if "Path efficiency:" in line:
                        match = re.search(r'Path efficiency:\s*([\d.]+)%', line)
                        if match:
                            self.path_efficiency = float(match.group(1))

            self.end_time = time.time()
            process.wait()

        except Exception as e:
            self.error_message = str(e)
            self.end_time = time.time()
            print(f"❌ Error: {e}")

        # Generate result summary
        return self._generate_result()

    def _generate_result(self) -> Dict:
        """Generate result dictionary."""
        duration = (self.end_time - self.start_time) if self.end_time and self.start_time else 0

        result = {
            "test_id": self.test_id,
            "goal": self.goal_position,
            "success": self.success,
            "timeout": self.timeout,
            "error": self.error_message,
            "iterations": self.iterations,
            "duration_seconds": round(duration, 2),
            "distance_traveled": round(self.distance_traveled, 2) if self.distance_traveled else None,
            "final_distance_to_goal": round(self.final_distance_to_goal, 2) if self.final_distance_to_goal else None,
            "path_efficiency": round(self.path_efficiency, 1) if self.path_efficiency else None,
        }

        # Print result
        status = "✅ SUCCESS" if self.success else ("⏱️ TIMEOUT" if self.timeout else "❌ FAILED")
        print(f"\n{status}")
        print(f"  Duration: {duration:.1f}s | Iterations: {self.iterations} | Distance: {self.final_distance_to_goal:.2f}m")
        if self.path_efficiency:
            print(f"  Path Efficiency: {self.path_efficiency:.1f}%")

        return result


class TestSuiteRunner:
    """Manages execution of multiple simulator tests."""

    def __init__(self, num_tests: int = 100, max_time_per_test: float = 120.0):
        self.num_tests = num_tests
        self.max_time_per_test = max_time_per_test
        self.results: List[Dict] = []
        self.start_time = None
        self.end_time = None

    def generate_goal_positions(self) -> List[Tuple[float, float, float]]:
        """Generate 100 diverse goal positions using values from 1 to 25."""
        goals = []
        random.seed(42)  # For reproducibility

        # Strategy: Mix of different distance ranges and axis combinations

        # 1. Close goals (1-5m on each axis) - 20 tests
        for _ in range(20):
            goals.append((
                random.uniform(1, 5),
                random.uniform(1, 5),
                random.uniform(1, 5)
            ))

        # 2. Medium goals (5-15m on each axis) - 30 tests
        for _ in range(30):
            goals.append((
                random.uniform(5, 15),
                random.uniform(5, 15),
                random.uniform(3, 10)  # Keep altitude reasonable
            ))

        # 3. Far goals (15-25m on each axis) - 25 tests
        for _ in range(25):
            goals.append((
                random.uniform(15, 25),
                random.uniform(15, 25),
                random.uniform(3, 15)
            ))

        # 4. Axis-aligned goals (test cardinal directions) - 15 tests
        for _ in range(5):
            dist = random.uniform(5, 20)
            goals.append((dist, 0.5, 3.0))  # Eastward
        for _ in range(5):
            dist = random.uniform(5, 20)
            goals.append((0.5, dist, 3.0))  # Northward
        for _ in range(5):
            dist = random.uniform(5, 15)
            goals.append((5.0, 5.0, dist))  # Upward

        # 5. Corner cases - 10 tests
        goals.extend([
            (1.0, 1.0, 1.0),   # Very close
            (25.0, 25.0, 15.0), # Very far
            (1.0, 25.0, 5.0),   # East far, North close
            (25.0, 1.0, 5.0),   # East close, North far
            (10.0, 10.0, 1.0),  # Low altitude
            (10.0, 10.0, 15.0), # High altitude
            (5.0, 5.0, 5.0),    # Equal axes
            (20.0, 5.0, 3.0),   # Elongated East
            (5.0, 20.0, 3.0),   # Elongated North
            (12.5, 12.5, 8.0),  # Middle distance
        ])

        return goals[:self.num_tests]

    def run_all_tests(self):
        """Execute all tests in the suite."""
        print("\n" + "="*80)
        print("🚀 SWARM AI SIMULATOR - AUTOMATED TEST SUITE")
        print("="*80)
        print(f"Tests to run: {self.num_tests}")
        print(f"Max time per test: {self.max_time_per_test}s")
        print(f"Start time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("="*80)

        goals = self.generate_goal_positions()
        self.start_time = time.time()

        for i, goal in enumerate(goals, start=1):
            test = SimulatorTest(i, goal, self.max_time_per_test)
            result = test.run()
            self.results.append(result)

            # Brief pause between tests
            time.sleep(2)

        self.end_time = time.time()

    def generate_report(self) -> str:
        """Generate comprehensive test report."""
        total_duration = self.end_time - self.start_time

        # Calculate statistics
        total_tests = len(self.results)
        successful = sum(1 for r in self.results if r["success"])
        timeouts = sum(1 for r in self.results if r["timeout"])
        failures = total_tests - successful - timeouts

        success_rate = (successful / total_tests * 100) if total_tests > 0 else 0

        # Collect metrics from successful tests
        successful_results = [r for r in self.results if r["success"]]

        avg_duration = sum(r["duration_seconds"] for r in successful_results) / len(successful_results) if successful_results else 0
        avg_iterations = sum(r["iterations"] for r in successful_results) / len(successful_results) if successful_results else 0
        avg_efficiency = sum(r["path_efficiency"] for r in successful_results if r["path_efficiency"]) / len([r for r in successful_results if r["path_efficiency"]]) if successful_results else 0

        # Build report
        report_lines = [
            "\n" + "="*80,
            "📊 TEST SUITE REPORT",
            "="*80,
            f"Execution Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
            f"Total Duration: {total_duration/60:.1f} minutes ({total_duration:.0f} seconds)",
            "",
            "OVERALL RESULTS:",
            f"  Total Tests: {total_tests}",
            f"  ✅ Successful: {successful} ({success_rate:.1f}%)",
            f"  ⏱️  Timeouts: {timeouts}",
            f"  ❌ Failures: {failures}",
            "",
            "PERFORMANCE METRICS (Successful Tests Only):",
            f"  Average Duration: {avg_duration:.1f}s",
            f"  Average Iterations: {avg_iterations:.0f}",
            f"  Average Path Efficiency: {avg_efficiency:.1f}%",
            "",
            "="*80,
            "DETAILED RESULTS:",
            "="*80,
        ]

        # Sort by success, then by test_id
        sorted_results = sorted(self.results, key=lambda x: (not x["success"], x["test_id"]))

        for r in sorted_results:
            status = "✅" if r["success"] else ("⏱️" if r["timeout"] else "❌")
            goal_str = f"({r['goal'][0]:.1f}, {r['goal'][1]:.1f}, {r['goal'][2]:.1f})"

            line = f"{status} Test {r['test_id']:03d} | Goal: {goal_str:20s} | "

            if r["success"]:
                line += f"{r['duration_seconds']:5.1f}s | {r['iterations']:4d} iters | "
                line += f"Final dist: {r['final_distance_to_goal']:5.2f}m"
                if r["path_efficiency"]:
                    line += f" | Eff: {r['path_efficiency']:5.1f}%"
            elif r["timeout"]:
                line += f"TIMEOUT after {r['duration_seconds']:.0f}s | Final dist: {r['final_distance_to_goal']:.2f}m"
            else:
                line += f"FAILED: {r['error'] if r['error'] else 'Unknown error'}"

            report_lines.append(line)

        report_lines.extend([
            "="*80,
            "",
            "💾 Detailed results saved to: test_results.json",
            ""
        ])

        return "\n".join(report_lines)

    def save_results(self, filename: str = "test_results.json"):
        """Save detailed results to JSON file."""
        output = {
            "test_suite": {
                "total_tests": len(self.results),
                "successful": sum(1 for r in self.results if r["success"]),
                "timeouts": sum(1 for r in self.results if r["timeout"]),
                "failures": len(self.results) - sum(1 for r in self.results if r["success"] or r["timeout"]),
                "total_duration_seconds": round(self.end_time - self.start_time, 2),
                "timestamp": datetime.now().isoformat(),
            },
            "results": self.results
        }

        filepath = Path(__file__).parent / filename
        with open(filepath, 'w') as f:
            json.dump(output, f, indent=2)

        print(f"✅ Results saved to: {filepath}")


def main():
    parser = argparse.ArgumentParser(description="Run automated simulator tests")
    parser.add_argument("--tests", type=int, default=100, help="Number of tests to run (default: 100)")
    parser.add_argument("--max-time", type=float, default=120.0, help="Max time per test in seconds (default: 120)")
    parser.add_argument("--output", type=str, default="test_results.json", help="Output JSON filename")

    args = parser.parse_args()

    # Run test suite
    runner = TestSuiteRunner(num_tests=args.tests, max_time_per_test=args.max_time)
    runner.run_all_tests()

    # Generate and display report
    report = runner.generate_report()
    print(report)

    # Save results
    runner.save_results(args.output)

    print("\n✨ Test suite completed!")


if __name__ == "__main__":
    main()
