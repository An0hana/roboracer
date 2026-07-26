#!/usr/bin/env python3
"""Obstacle-injection acceptance test for the MPPI simulation stack.

Runs three phases against a live closed-loop sim and prints a JSON verdict:

  A  avoid:  a static obstacle sits on the race line; the car must keep
             lapping with positive obstacle clearance and no stops.
  B  stale:  publishing stops; diagnostics must flag obstacles_stale=true
             within the timeout while the car keeps driving.
  C  block:  a wall of obstacles closes the whole corridor ahead of the car;
             the car must brake to a stop without ever reporting a collision.

    python3 sim_obstacle_test.py --raceline raceline.csv --output verdict.json
"""

import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from roboracer_msgs.msg import TrackedObstacle, TrackedObstacleArray
from visualization_msgs.msg import Marker, MarkerArray


def load_raceline(path):
    points = []
    with open(path, encoding="utf-8") as handle:
        header = handle.readline().strip().split(",")
        idx = {name: header.index(name) for name in ("s", "x", "y", "yaw")}
        for line in handle:
            cells = line.strip().split(",")
            if len(cells) < 4:
                continue
            points.append(tuple(float(cells[idx[name]])
                                for name in ("s", "x", "y", "yaw")))
    return points


class ObstacleTest(Node):
    def __init__(self, args):
        super().__init__("sim_obstacle_test")
        self.args = args
        self.raceline = load_raceline(args.raceline)
        self.publisher = self.create_publisher(
            TrackedObstacleArray, args.obstacles_topic, 1)
        # Red boxes on the topic the stock RViz config already displays, so a
        # human can see where the injected obstacles are.
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/perception/obstacle_markers", 1)
        self.odom = None
        self.diags = []
        self.create_subscription(Odometry, args.odom_topic, self.on_odom, 10)
        self.create_subscription(DiagnosticArray, "/diagnostics", self.on_diag, 10)

    def on_odom(self, message):
        self.odom = (message.pose.pose.position.x, message.pose.pose.position.y,
                     message.twist.twist.linear.x)

    def on_diag(self, message):
        for status in message.status:
            if "controller" not in status.name:
                continue
            values = {kv.key: kv.value for kv in status.values}
            values["_message"] = status.message
            values["_time"] = time.monotonic()
            self.diags.append(values)

    def spin_for(self, seconds, publish=None, period=0.05):
        end = time.monotonic() + seconds
        next_publish = 0.0
        while time.monotonic() < end:
            if publish is not None and time.monotonic() >= next_publish:
                publish()
                next_publish = time.monotonic() + period
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for_odom(self, seconds=10.0):
        end = time.monotonic() + seconds
        while self.odom is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.odom is not None

    def raceline_point_ahead(self, distance):
        x, y, _ = self.odom
        best_index = min(
            range(len(self.raceline)),
            key=lambda i: (self.raceline[i][1] - x) ** 2 +
                          (self.raceline[i][2] - y) ** 2)
        target_s = self.raceline[best_index][0] + distance
        total = self.raceline[-1][0]
        target_s = math.fmod(target_s, total)
        return min(self.raceline,
                   key=lambda p: abs(p[0] - target_s))

    def raceline_point_at(self, target_s):
        return min(self.raceline, key=lambda p: abs(p[0] - target_s))

    def make_obstacle(self, x, y, yaw, identity):
        obstacle = TrackedObstacle()
        obstacle.id = identity
        obstacle.classification = TrackedObstacle.STATIC_OBSTACLE
        obstacle.x = x
        obstacle.y = y
        obstacle.yaw = yaw
        obstacle.length = 0.5
        obstacle.width = 0.3
        obstacle.confidence = 0.9
        obstacle.dynamic = False
        obstacle.visible = True
        return obstacle

    def publish_set(self, obstacles):
        message = TrackedObstacleArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.obstacles = obstacles
        self.publisher.publish(message)
        markers = MarkerArray()
        for obstacle in obstacles:
            marker = Marker()
            marker.header = message.header
            marker.ns = "sim_obstacle_test"
            marker.id = obstacle.id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = obstacle.x
            marker.pose.position.y = obstacle.y
            marker.pose.position.z = 0.15
            marker.pose.orientation.z = math.sin(obstacle.yaw * 0.5)
            marker.pose.orientation.w = math.cos(obstacle.yaw * 0.5)
            marker.scale.x = obstacle.length
            marker.scale.y = obstacle.width
            marker.scale.z = 0.3
            marker.color.r = 1.0
            marker.color.a = 0.9
            marker.lifetime.sec = 1
            markers.markers.append(marker)
        self.marker_publisher.publish(markers)

    def diag_window(self, since):
        return [d for d in self.diags if d["_time"] >= since]

    @staticmethod
    def floats(diags, key):
        values = []
        for entry in diags:
            raw = entry.get(key)
            if raw is None:
                continue
            try:
                value = float(raw)
            except ValueError:
                continue
            if math.isfinite(value):
                values.append(value)
        return values

    def run(self):
        result = {}
        fails = []
        if not self.wait_for_odom():
            print(json.dumps({"pass": False,
                              "fail_reasons": ["no odometry"]}))
            return 1

        # Phase A — static obstacle on the race line, car keeps lapping.
        if self.args.obstacle_s >= 0.0:
            anchor = self.raceline_point_at(self.args.obstacle_s)
        else:
            anchor = self.raceline_point_ahead(self.args.obstacle_ahead)
        yaw = anchor[3]
        lateral = (math.cos(yaw + math.pi / 2.0), math.sin(yaw + math.pi / 2.0))
        static_set = [self.make_obstacle(
            anchor[1] + lateral[0] * self.args.obstacle_offset,
            anchor[2] + lateral[1] * self.args.obstacle_offset, yaw, 1)]
        start = time.monotonic()
        self.spin_for(self.args.avoid_duration,
                      publish=lambda: self.publish_set(static_set))
        window = self.diag_window(start)
        seen = self.floats(window, "obstacle_count")
        clearances = self.floats(window, "minimum_obstacle_clearance_m")
        ok_ticks = sum(1 for d in window if d["_message"].startswith("ok"))
        result["avoid"] = {
            "ticks": len(window),
            "ok_ticks": ok_ticks,
            "obstacle_seen_ticks": sum(1 for v in seen if v >= 1.0),
            "min_obstacle_clearance_m":
                round(min(clearances), 3) if clearances else None,
            "end_speed": round(self.odom[2], 2)}
        if not window:
            fails.append("avoid: no diagnostics")
        else:
            if ok_ticks / len(window) < 0.90:
                fails.append(f"avoid: ok ratio {ok_ticks / len(window):.2f}")
            if not any(v >= 1.0 for v in seen):
                fails.append("avoid: controller never saw the obstacle")
            if clearances and min(clearances) < 0.0:
                fails.append(f"avoid: clearance {min(clearances):.3f} negative")
            if self.odom[2] < 0.3:
                fails.append("avoid: car ended nearly stopped")

        if self.args.skip_stale and self.args.skip_block:
            result["fail_reasons"] = fails
            result["pass"] = not fails
            text = json.dumps(result, indent=2, ensure_ascii=False)
            print(text)
            if self.args.output:
                with open(self.args.output, "w", encoding="utf-8") as handle:
                    handle.write(text + "\n")
            return 0 if not fails else 1

        # Phase B — stop publishing; stale flag must rise, car keeps moving.
        start = time.monotonic()
        self.spin_for(self.args.stale_duration)
        window = self.diag_window(start + self.args.stale_settle)
        stale = [d.get("obstacles_stale") for d in window]
        moving = self.floats(window, "solve_time_ms")
        result["stale"] = {
            "ticks": len(window),
            "stale_true_ticks": stale.count("true"),
            "end_speed": round(self.odom[2], 2)}
        if not window:
            fails.append("stale: no diagnostics")
        else:
            if stale.count("true") < len(window) * 0.8:
                fails.append("stale: obstacles_stale did not latch")
            if not moving or self.odom[2] < 0.3:
                fails.append("stale: car stopped on stale obstacles")

        # Phase C — wall across the corridor ahead; car must stop cleanly.
        anchor = self.raceline_point_ahead(self.args.block_ahead)
        yaw = anchor[3]
        normal = (math.cos(yaw + math.pi / 2.0), math.sin(yaw + math.pi / 2.0))
        wall = []
        for index, offset in enumerate((-1.2, -0.6, 0.0, 0.6, 1.2)):
            wall.append(self.make_obstacle(
                anchor[1] + normal[0] * offset,
                anchor[2] + normal[1] * offset, yaw, 10 + index))
        start = time.monotonic()
        self.spin_for(self.args.block_duration,
                      publish=lambda: self.publish_set(wall))
        window = self.diag_window(start)
        clearances = self.floats(window, "minimum_obstacle_clearance_m")
        collisions = self.floats(window, "cost_collision")
        speeds_at_end = self.odom[2]
        stop_reasons = {}
        for entry in window:
            reason = entry["_message"]
            stop_reasons[reason] = stop_reasons.get(reason, 0) + 1
        result["block"] = {
            "ticks": len(window),
            "reasons": stop_reasons,
            "min_obstacle_clearance_m":
                round(min(clearances), 3) if clearances else None,
            "end_speed": round(speeds_at_end, 2)}
        if abs(speeds_at_end) > 0.15:
            fails.append(f"block: car still moving at {speeds_at_end:.2f} m/s")
        if clearances and min(clearances) < 0.0:
            fails.append(f"block: clearance {min(clearances):.3f} negative "
                         "(drove into the wall)")
        if any(value > 0.0 for value in collisions):
            fails.append("block: published trajectory carried collision cost")

        result["fail_reasons"] = fails
        result["pass"] = not fails
        text = json.dumps(result, indent=2, ensure_ascii=False)
        print(text)
        if self.args.output:
            with open(self.args.output, "w", encoding="utf-8") as handle:
                handle.write(text + "\n")
        return 0 if not fails else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raceline", required=True)
    parser.add_argument("--odom-topic", default="/ego_racecar/odom")
    parser.add_argument("--obstacles-topic", default="/perception/obstacles")
    parser.add_argument("--obstacle-ahead", type=float, default=10.0)
    parser.add_argument("--obstacle-s", type=float, default=-1.0,
                        help="place phase-A obstacle at this absolute race-line s")
    parser.add_argument("--obstacle-offset", type=float, default=0.0,
                        help="lateral offset of the phase-A obstacle (left positive)")
    parser.add_argument("--skip-block", action="store_true")
    parser.add_argument("--skip-stale", action="store_true")
    parser.add_argument("--block-ahead", type=float, default=8.0)
    parser.add_argument("--avoid-duration", type=float, default=45.0)
    parser.add_argument("--stale-duration", type=float, default=8.0)
    parser.add_argument("--stale-settle", type=float, default=1.0)
    parser.add_argument("--block-duration", type=float, default=25.0)
    parser.add_argument("--output", default="")
    args = parser.parse_args()

    rclpy.init()
    node = ObstacleTest(args)
    code = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
