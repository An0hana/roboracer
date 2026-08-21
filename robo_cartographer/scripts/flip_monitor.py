#!/usr/bin/env python3
"""Live flip monitor. Run alongside the stack, drive a couple of laps, read the
summary at Ctrl-C.

    ros2 run robo_cartographer flip_monitor.py
    # or, without installing:
    python3 flip_monitor.py

WHY
---
"It flipped severely" is not measurable. This prints the same numbers the bag
analysis produced, live:

  * map->odom is the localizer's correction term. Between corrections it is
    exactly constant, so every change is a discrete pose-graph event.
  * A step is reported with its size, the vehicle's map position when it
    happened, and the direction of the step in the map frame.
  * At exit you get the histogram, the biggest step, the net drift, and -- the
    part that matters -- whether steps cluster at particular track locations
    or in a particular direction.

The bag showed x-axis steps of 0.585 m and 0.618 m, 19 steps > 2 cm total, and
a net map->odom drift of 0.19 m / 4.2 deg over 169 s. Compare against that.

Save the JSON it writes and the numbers are directly comparable between runs.
"""

import json
import math
import os
import signal
import sys
import time
from collections import Counter

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from tf2_msgs.msg import TFMessage

# Report a correction once it exceeds this. 2 cm matches the bag analysis.
STEP_THRESHOLD_M = 0.02
STEP_THRESHOLD_RAD = math.radians(1.0)

# Steps at or above this are the failure mode being hunted.
FLIP_THRESHOLD_M = 0.25


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class FlipMonitor(Node):
    def __init__(self):
        super().__init__("flip_monitor")

        # /tf is published BEST_EFFORT by some stacks and RELIABLE by others.
        # tf2_ros uses RELIABLE with a deep queue, so match that.
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=200)
        self.create_subscription(TFMessage, "/tf", self.on_tf, qos)

        self.t0 = None
        self.mo = None          # last map->odom
        self.mo_first = None
        self.ob = None          # last odom->base_link, for reporting position
        self.steps = []
        self.n_mo = 0

        self.get_logger().info(
            "watching map->odom. Drive at least two laps, then Ctrl-C.")

    def on_tf(self, msg):
        for tr in msg.transforms:
            t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            if self.t0 is None:
                self.t0 = t

            if tr.child_frame_id == "base_link" and tr.header.frame_id == "odom":
                self.ob = (tr.transform.translation.x,
                           tr.transform.translation.y,
                           yaw_of(tr.transform.rotation))

            elif tr.child_frame_id == "odom" and tr.header.frame_id == "map":
                cur = (tr.transform.translation.x,
                       tr.transform.translation.y,
                       yaw_of(tr.transform.rotation))
                self.n_mo += 1
                if self.mo_first is None:
                    self.mo_first = cur
                if self.mo is not None:
                    self.check_step(t, self.mo, cur)
                self.mo = cur

    def check_step(self, t, prev, cur):
        dx, dy = cur[0] - prev[0], cur[1] - prev[1]
        d = math.hypot(dx, dy)
        dyaw = wrap(cur[2] - prev[2])
        if d < STEP_THRESHOLD_M and abs(dyaw) < STEP_THRESHOLD_RAD:
            return

        # Where was the car when the correction landed?
        px = py = float("nan")
        if self.ob is not None:
            c, s = math.cos(cur[2]), math.sin(cur[2])
            px = cur[0] + c * self.ob[0] - s * self.ob[1]
            py = cur[1] + s * self.ob[0] + c * self.ob[1]

        rel = t - self.t0
        step = {
            "t": round(rel, 2),
            "step_m": round(d, 4),
            "step_deg": round(math.degrees(dyaw), 3),
            "dir_deg": round(math.degrees(math.atan2(dy, dx)) % 360.0, 1),
            "at_x": round(px, 2),
            "at_y": round(py, 2),
        }
        self.steps.append(step)

        tag = "FLIP" if d >= FLIP_THRESHOLD_M else "step"
        line = ("[{:7.2f}s] {} {:.3f} m  dyaw {:+.2f} deg  "
                "dir {:5.1f} deg  at ({:.2f}, {:.2f})").format(
                    rel, tag, d, math.degrees(dyaw), step["dir_deg"], px, py)
        if d >= FLIP_THRESHOLD_M:
            self.get_logger().error(line)
        else:
            self.get_logger().info(line)

    def summary(self):
        out = []
        add = out.append
        add("")
        add("=" * 68)
        add("FLIP MONITOR SUMMARY")
        add("=" * 68)

        if self.mo is None or self.t0 is None:
            add("No map->odom transforms seen. Either the localizer never")
            add("started a trajectory, or nothing is publishing /tf.")
            return "\n".join(out), {}

        dur = 0.0
        if self.steps:
            dur = self.steps[-1]["t"]
        add("map->odom messages : {}".format(self.n_mo))
        add("corrections > {:.0f} cm : {}".format(
            STEP_THRESHOLD_M * 100, len(self.steps)))

        sizes = [s["step_m"] for s in self.steps]
        flips = [s for s in self.steps if s["step_m"] >= FLIP_THRESHOLD_M]
        add("corrections >= {:.2f} m (FLIPS) : {}".format(
            FLIP_THRESHOLD_M, len(flips)))

        if sizes:
            sizes_sorted = sorted(sizes)
            add("step size  max {:.3f} m   median {:.3f} m".format(
                sizes_sorted[-1], sizes_sorted[len(sizes_sorted) // 2]))
            buckets = Counter()
            for v in sizes:
                if v < 0.05:
                    buckets["<5cm"] += 1
                elif v < 0.15:
                    buckets["5-15cm"] += 1
                elif v < 0.25:
                    buckets["15-25cm"] += 1
                elif v < 0.5:
                    buckets["25-50cm"] += 1
                else:
                    buckets[">50cm"] += 1
            add("histogram : " + "  ".join(
                "{}={}".format(k, buckets[k]) for k in
                ["<5cm", "5-15cm", "15-25cm", "25-50cm", ">50cm"]
                if buckets[k]))

        net_x = self.mo[0] - self.mo_first[0]
        net_y = self.mo[1] - self.mo_first[1]
        net_yaw = math.degrees(wrap(self.mo[2] - self.mo_first[2]))
        add("net map->odom drift : dx={:+.3f} dy={:+.3f} dyaw={:+.2f} deg".format(
            net_x, net_y, net_yaw))

        if flips:
            add("")
            add("FLIPS -- these are the failure mode:")
            for s in flips:
                add("  t={:7.2f}s  {:.3f} m  dir {:5.1f} deg  at ({:.2f}, {:.2f})"
                    .format(s["t"], s["step_m"], s["dir_deg"],
                            s["at_x"], s["at_y"]))

            # Direction clustering: the bag's flips were both along x.
            dirs = [s["dir_deg"] for s in flips]
            axis = Counter()
            for d in dirs:
                a = d % 180.0
                if a < 30 or a >= 150:
                    axis["along x"] += 1
                elif 60 <= a < 120:
                    axis["along y"] += 1
                else:
                    axis["diagonal"] += 1
            add("  direction : " + ", ".join(
                "{} {}".format(v, k) for k, v in axis.most_common()))

            # Location clustering: repeated flips at one place mean an
            # ambiguous spot in the map, not a global tuning problem.
            cells = Counter()
            for s in flips:
                if not math.isnan(s["at_x"]):
                    cells[(round(s["at_x"] / 2) * 2,
                           round(s["at_y"] / 2) * 2)] += 1
            repeats = [(c, n) for c, n in cells.items() if n > 1]
            if repeats:
                add("  REPEATED at : " + ", ".join(
                    "{} x{}".format(c, n) for c, n in repeats))
                add("  -> a specific place in the map is ambiguous. Fixing that")
                add("     stretch of map beats any further parameter tuning.")
            else:
                add("  no location repeats -- spread across the track.")
        else:
            add("")
            add("No flips. If the pose still felt wrong, the error is")
            add("continuous drift rather than discrete jumps, which is a")
            add("different problem with a different fix.")

        add("=" * 68)

        data = {
            "duration_s": dur,
            "n_map_odom_msgs": self.n_mo,
            "n_steps": len(self.steps),
            "n_flips": len(flips),
            "max_step_m": max(sizes) if sizes else 0.0,
            "net_drift": {"dx": net_x, "dy": net_y, "dyaw_deg": net_yaw},
            "steps": self.steps,
        }
        return "\n".join(out), data


def main():
    rclpy.init()
    node = FlipMonitor()

    def finish(*_):
        text, data = node.summary()
        print(text)
        if data:
            path = os.path.join(
                "/tmp", "flip_monitor_{}.json".format(int(time.time())))
            with open(path, "w") as fh:
                json.dump(data, fh, indent=2)
            print("wrote {}".format(path))
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT, finish)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        finish()


if __name__ == "__main__":
    main()
