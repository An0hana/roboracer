#!/usr/bin/env python3
"""Closed-loop acceptance monitor for the scan-costmap MPPI simulation stack.

Subscribes to odometry, drive commands and controller diagnostics for a fixed
window, projects the vehicle onto the race line, and prints a JSON verdict:

    python3 sim_acceptance_monitor.py --duration 60 --target-speed 0.5 \
        --raceline /path/raceline.csv --output verdict.json

Verdict fields cover the step-2 acceptance criteria: sustained forward
progress, speed tracking, solver health (rate, timing, reasons, NaN), and
minimum clearances. The script is deliberately read-only — it never publishes.
"""

import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry


def load_raceline(path):
    points = []
    with open(path, encoding="utf-8") as handle:
        header = handle.readline().strip().split(",")
        index_s = header.index("s")
        index_x = header.index("x")
        index_y = header.index("y")
        for line in handle:
            cells = line.strip().split(",")
            if len(cells) < 3:
                continue
            points.append((float(cells[index_s]), float(cells[index_x]),
                           float(cells[index_y])))
    if len(points) < 10:
        sys.exit(f"race line {path} has too few points")
    length = points[-1][0] + math.hypot(
        points[0][1] - points[-1][1], points[0][2] - points[-1][2])
    return points, length


class Monitor(Node):
    def __init__(self, args):
        super().__init__("sim_acceptance_monitor")
        self.args = args
        self.raceline, self.track_length = load_raceline(args.raceline)
        self.odom_samples = []
        self.drive_samples = []
        self.diag_samples = []
        self.nan_hits = []
        best_effort = QoSProfile(depth=10)
        best_effort.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Odometry, args.odom_topic, self.on_odom, 10)
        self.create_subscription(
            AckermannDriveStamped, args.drive_topic, self.on_drive, 10)
        self.create_subscription(
            DiagnosticArray, "/diagnostics", self.on_diag, 10)

    def on_odom(self, message):
        self.odom_samples.append((
            time.monotonic(),
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.twist.twist.linear.x))

    def on_drive(self, message):
        self.drive_samples.append((
            time.monotonic(), message.drive.speed, message.drive.steering_angle,
            message.drive.acceleration))

    def on_diag(self, message):
        for status in message.status:
            if "controller" not in status.name:
                continue
            values = {kv.key: kv.value for kv in status.values}
            values["_message"] = status.message
            values["_level"] = int.from_bytes(status.level, "little") if isinstance(
                status.level, bytes) else int(status.level)
            self.diag_samples.append(values)
            for key, value in values.items():
                if not isinstance(value, str):
                    continue
                lowered = value.lower()
                # NaN is always a defect. Infinity is the legitimate
                # "not applicable" value for clearances (no obstacle in range)
                # and for ages of inputs that have not arrived yet.
                if "nan" in lowered:
                    if key not in ("obstacle_age_s", "costmap_age_s"):
                        self.nan_hits.append(key)
                elif "inf" in lowered and "clearance" not in key and \
                        not key.endswith("_age_s"):
                    self.nan_hits.append(key)

    def nearest_s(self, x, y):
        best = None
        best_distance = float("inf")
        for s, px, py in self.raceline:
            distance = (px - x) ** 2 + (py - y) ** 2
            if distance < best_distance:
                best_distance = distance
                best = s
        return best, math.sqrt(best_distance)

    def verdict(self):
        result = {"duration_s": self.args.duration,
                  "target_speed": self.args.target_speed,
                  "odom_count": len(self.odom_samples),
                  "drive_count": len(self.drive_samples),
                  "diag_count": len(self.diag_samples)}
        if not self.odom_samples:
            result["pass"] = False
            result["fail_reasons"] = ["no odometry received"]
            return result

        # Forward progress with wrap handling.
        progress = 0.0
        deviations = []
        previous_s = None
        for _, x, y, _ in self.odom_samples[:: max(1, len(self.odom_samples) // 400)]:
            s, deviation = self.nearest_s(x, y)
            deviations.append(deviation)
            if previous_s is not None:
                delta = s - previous_s
                if delta > self.track_length * 0.5:
                    delta -= self.track_length
                elif delta < -self.track_length * 0.5:
                    delta += self.track_length
                progress += delta
            previous_s = s
        speeds = [sample[3] for sample in self.odom_samples]
        settle = len(speeds) // 5
        cruise_speeds = speeds[settle:] or speeds
        result["forward_progress_m"] = round(progress, 2)
        result["laps"] = round(progress / self.track_length, 2)
        result["mean_speed"] = round(sum(cruise_speeds) / len(cruise_speeds), 3)
        result["max_speed"] = round(max(speeds), 3)
        result["max_raceline_deviation_m"] = round(max(deviations), 3)

        reasons = {}
        solve_times = []
        min_clearances = []
        for values in self.diag_samples:
            reason = values.get("_message", "unknown")
            reasons[reason] = reasons.get(reason, 0) + 1
            if "solve_time_ms" in values:
                try:
                    solve_times.append(float(values["solve_time_ms"]))
                except ValueError:
                    pass
            if "minimum_clearance_m" in values:
                try:
                    min_clearances.append(float(values["minimum_clearance_m"]))
                except ValueError:
                    pass
        result["reasons"] = reasons
        if solve_times:
            solve_times.sort()
            result["solve_time_ms"] = {
                "mean": round(sum(solve_times) / len(solve_times), 2),
                "p95": round(solve_times[int(len(solve_times) * 0.95) - 1], 2),
                "max": round(solve_times[-1], 2)}
        if min_clearances:
            result["min_clearance_m"] = round(min(min_clearances), 3)
        result["nan_keys"] = sorted(set(self.nan_hits))

        fail = []
        ok_ticks = sum(count for reason, count in reasons.items()
                       if reason.startswith("ok"))
        total_ticks = sum(reasons.values())
        if total_ticks:
            result["ok_ratio"] = round(ok_ticks / total_ticks, 3)
            if ok_ticks / total_ticks < self.args.min_ok_ratio:
                fail.append(f"ok ratio {ok_ticks / total_ticks:.2f} below "
                            f"{self.args.min_ok_ratio}")
        else:
            fail.append("no controller diagnostics received")
        expected = self.args.target_speed * self.args.duration * \
            self.args.min_progress_fraction
        if progress < expected:
            fail.append(f"progress {progress:.1f} m below {expected:.1f} m")
        if self.args.max_speed_limit and max(speeds) > self.args.max_speed_limit:
            fail.append(f"speed {max(speeds):.2f} exceeded limit "
                        f"{self.args.max_speed_limit:.2f}")
        if min_clearances and min(min_clearances) < 0.0:
            fail.append(f"clearance went negative: {min(min_clearances):.3f}")
        if self.nan_hits:
            fail.append(f"NaN in diagnostics keys: {sorted(set(self.nan_hits))}")
        if solve_times and solve_times[-1] > self.args.max_solve_ms:
            fail.append(f"solve time max {solve_times[-1]:.1f} ms over "
                        f"{self.args.max_solve_ms} ms")
        result["fail_reasons"] = fail
        result["pass"] = not fail
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--target-speed", type=float, default=0.5)
    parser.add_argument("--raceline", required=True)
    parser.add_argument("--odom-topic", default="/ego_racecar/odom")
    parser.add_argument("--drive-topic", default="/drive")
    parser.add_argument("--min-ok-ratio", type=float, default=0.95)
    parser.add_argument("--min-progress-fraction", type=float, default=0.5)
    parser.add_argument("--max-speed-limit", type=float, default=0.0,
                        help="fail if odom speed ever exceeds this (0 = off)")
    parser.add_argument("--max-solve-ms", type=float, default=45.0)
    parser.add_argument("--output", default="")
    args = parser.parse_args()

    rclpy.init()
    monitor = Monitor(args)
    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        rclpy.spin_once(monitor, timeout_sec=0.2)
    verdict = monitor.verdict()
    text = json.dumps(verdict, indent=2, ensure_ascii=False)
    print(text)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as handle:
            handle.write(text + "\n")
    monitor.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if verdict.get("pass") else 1)


if __name__ == "__main__":
    main()
