#!/usr/bin/env python3
"""Re-cap a race line's v_ref by corridor width and braking feasibility.

The stock generator caps reference speed by lateral acceleration only, which
happily demands 4 m/s through a 1.6 m corridor. This post-processor adds:

  1. width cap:    v <= base + gain * min(width_left, width_right)
  2. curvature cap (kept from the original v_ref, never raised)
  3. accel/brake feasibility: forward pass at a_accel, backward pass at
     a_brake over arc length, wrapped around the closed loop twice so the
     seam is consistent.

    python3 retune_raceline_speed.py in.csv out.csv \
        --width-base 1.2 --width-gain 1.6 --accel 1.0 --brake 1.5
"""

import argparse
import csv
import math
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input")
    parser.add_argument("output")
    parser.add_argument("--width-base", type=float, default=1.2,
                        help="speed allowed at zero corridor half-width")
    parser.add_argument("--width-gain", type=float, default=1.6,
                        help="extra speed per metre of narrow-side width")
    parser.add_argument("--accel", type=float, default=1.0)
    parser.add_argument("--brake", type=float, default=1.5)
    parser.add_argument("--floor", type=float, default=0.8,
                        help="never cap below this speed")
    args = parser.parse_args()

    with open(args.input, encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fields = reader.fieldnames
        rows = list(reader)
    if not rows or "v_ref" not in fields:
        sys.exit("input is not a raceline csv")

    count = len(rows)
    s = [float(r["s"]) for r in rows]
    speeds = []
    for row in rows:
        narrow = min(float(row["width_left"]), float(row["width_right"]))
        width_cap = max(args.floor, args.width_base + args.width_gain * narrow)
        speeds.append(min(float(row["v_ref"]), width_cap))

    def gap(i):
        nxt = (i + 1) % count
        d = s[nxt] - s[i]
        if d <= 0.0:
            d = math.hypot(float(rows[nxt]["x"]) - float(rows[i]["x"]),
                           float(rows[nxt]["y"]) - float(rows[i]["y"]))
        return max(d, 1.0e-6)

    # Two wraps make the closed-loop profile consistent across the seam.
    for _ in range(2):
        for i in range(count):  # forward: acceleration limit
            j = (i + 1) % count
            speeds[j] = min(speeds[j], math.sqrt(
                speeds[i] ** 2 + 2.0 * args.accel * gap(i)))
        for i in range(count - 1, -1, -1):  # backward: braking limit
            j = (i + 1) % count
            speeds[i] = min(speeds[i], math.sqrt(
                speeds[j] ** 2 + 2.0 * args.brake * gap(i)))

    changed = sum(1 for row, v in zip(rows, speeds)
                  if abs(float(row["v_ref"]) - v) > 1.0e-6)
    for row, v in zip(rows, speeds):
        row["v_ref"] = f"{v:.9f}"

    with open(args.output, "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print(f"rewrote {changed}/{count} v_ref values; "
          f"min={min(speeds):.2f} max={max(speeds):.2f} "
          f"mean={sum(speeds) / count:.2f}")


if __name__ == "__main__":
    main()
