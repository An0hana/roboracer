#!/usr/bin/env python3
"""Estimate vesc_ackermann's speed_to_erpm_gain from a calibration rosbag.

Record a bag while driving straight at a few constant ERPM setpoints:

    ros2 bag record -o erpm_calib /sensors/core /tracked_pose

Then:

    python3 calibrate_erpm_gain.py erpm_calib

The gain is ERPM per (m/s). Raw ERPM is read from /sensors/core, so the
result does not depend on the gain currently configured on the car.

Ground-truth speed comes from a least-squares fit of Cartographer position
against time over each constant-ERPM plateau. A straight fit is used rather
than summed point-to-point distances because pose noise inflates path length
but leaves the regression slope unbiased. Drive straight; R^2 reports how
well that held.
"""

import argparse
import math
import sys

import numpy as np

try:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import rosbag2_py
except ImportError as exc:  # pragma: no cover - environment dependent
    sys.exit(f"ROS 2 Python packages unavailable: {exc}\nRun with a sourced ROS 2 Humble environment.")


ERPM_TOPIC = "/sensors/core"
POSE_TOPIC = "/tracked_pose"


def read_bag(path, storage_id):
    """Return (erpm_samples, pose_samples) as lists of (stamp_seconds, value)."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for topic in (ERPM_TOPIC, POSE_TOPIC):
        if topic not in type_map:
            sys.exit(f"Bag does not contain {topic}. Found: {sorted(type_map)}")

    erpm, poses = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in (ERPM_TOPIC, POSE_TOPIC):
            continue
        message = deserialize_message(data, get_message(type_map[topic]))
        # Use the source stamp, never bag receive time: the two topics come
        # from different nodes and receive time would skew the fit.
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if topic == ERPM_TOPIC:
            erpm.append((stamp, message.state.speed))
        else:
            poses.append((stamp, message.pose.position.x, message.pose.position.y))
    return erpm, poses


def find_plateaus(erpm, min_erpm, tolerance, min_duration):
    """Split ERPM samples into runs that hold steady within `tolerance` (fraction)."""
    plateaus = []
    start = None
    for index, (stamp, value) in enumerate(erpm):
        if abs(value) < min_erpm:
            start = None
            continue
        if start is None:
            start = index
            continue
        window = [v for _, v in erpm[start:index + 1]]
        reference = float(np.median(window))
        if abs(value - reference) > abs(reference) * tolerance:
            if erpm[index - 1][0] - erpm[start][0] >= min_duration:
                plateaus.append((start, index - 1))
            start = index
    if start is not None and erpm[-1][0] - erpm[start][0] >= min_duration:
        plateaus.append((start, len(erpm) - 1))
    return plateaus


def fit_speed(poses, t_start, t_end):
    """Least-squares speed over [t_start, t_end]. Returns (speed, r_squared, n)."""
    window = [p for p in poses if t_start <= p[0] <= t_end]
    if len(window) < 10:
        return None, None, len(window)
    times = np.array([p[0] for p in window]) - t_start
    xs = np.array([p[1] for p in window])
    ys = np.array([p[2] for p in window])

    slopes, residual_sum = [], 0.0
    total_variance = 0.0
    for values in (xs, ys):
        slope, intercept = np.polyfit(times, values, 1)
        slopes.append(slope)
        residual_sum += float(np.sum((values - (slope * times + intercept)) ** 2))
        total_variance += float(np.sum((values - np.mean(values)) ** 2))
    r_squared = 1.0 - residual_sum / total_variance if total_variance > 0.0 else 0.0
    return math.hypot(*slopes), r_squared, len(window)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", help="Path to the rosbag2 directory")
    parser.add_argument("--storage", default="sqlite3", help="rosbag2 storage id")
    parser.add_argument("--min-erpm", type=float, default=500.0,
                        help="Ignore samples below this magnitude (default: 500)")
    parser.add_argument("--tolerance", type=float, default=0.05,
                        help="Plateau tolerance as a fraction of ERPM (default: 0.05)")
    parser.add_argument("--min-duration", type=float, default=3.0,
                        help="Minimum plateau duration in seconds (default: 3.0)")
    parser.add_argument("--trim", type=float, default=0.5,
                        help="Seconds discarded at each plateau edge (default: 0.5)")
    args = parser.parse_args()

    erpm, poses = read_bag(args.bag, args.storage)
    if not erpm or not poses:
        sys.exit("Bag contains no usable samples on both topics.")
    print(f"Loaded {len(erpm)} ERPM samples and {len(poses)} poses.\n")

    plateaus = find_plateaus(erpm, args.min_erpm, args.tolerance, args.min_duration)
    if not plateaus:
        sys.exit("No steady ERPM plateau found. Hold each setpoint longer, "
                 "or relax --tolerance / --min-duration.")

    print(f"{'ERPM':>10} {'speed m/s':>11} {'gain':>10} {'R^2':>8} {'dur s':>7} {'poses':>7}")
    print("-" * 58)
    gains = []
    for first, last in plateaus:
        t_start = erpm[first][0] + args.trim
        t_end = erpm[last][0] - args.trim
        if t_end <= t_start:
            continue
        window = [v for t, v in erpm[first:last + 1] if t_start <= t <= t_end]
        mean_erpm = float(np.mean(window)) if window else 0.0
        speed, r_squared, count = fit_speed(poses, t_start, t_end)
        if speed is None or speed < 0.05:
            print(f"{mean_erpm:>10.0f} {'--':>11} {'--':>10} {'--':>8} "
                  f"{t_end - t_start:>7.1f} {count:>7}   (too few poses / no motion)")
            continue
        gain = mean_erpm / speed
        gains.append(gain)
        print(f"{mean_erpm:>10.0f} {speed:>11.3f} {gain:>10.1f} {r_squared:>8.4f} "
              f"{t_end - t_start:>7.1f} {count:>7}")

    if not gains:
        sys.exit("\nNo plateau produced a usable speed estimate.")

    mean_gain = float(np.mean(gains))
    spread = (max(gains) - min(gains)) / mean_gain if mean_gain else 0.0
    print("-" * 58)
    print(f"\nspeed_to_erpm_gain: {mean_gain:.1f}")
    print(f"spread across plateaus: {spread * 100.0:.1f}%")

    if spread > 0.05:
        print("\nWARNING: plateaus disagree by more than 5%. The relationship is not a\n"
              "pure gain -- check for wheel slip, or calibrate speed_to_erpm_offset too.")
    if len(gains) < 2:
        print("\nNOTE: only one plateau. Run at 2-3 different speeds to confirm linearity.")


if __name__ == "__main__":
    main()
