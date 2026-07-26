#!/usr/bin/env python3
"""Generate the strict mppi_controller race-line CSV from a closed x/y centerline."""

import argparse
import csv
import math
import sys
from pathlib import Path


OUTPUT_FIELDS = (
    "s", "x", "y", "yaw", "curvature", "v_ref", "width_left", "width_right"
)


def finite_number(value: str, field: str, row: int) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"row {row}: {field} is not a number") from exc
    if not math.isfinite(number):
        raise ValueError(f"row {row}: {field} must be finite")
    return number


def read_centerline(path: Path, defaults: dict[str, float]) -> list[dict[str, float]]:
    with path.open(newline="", encoding="utf-8-sig") as source:
        reader = csv.DictReader(source)
        if reader.fieldnames is None or not {"x", "y"}.issubset(reader.fieldnames):
            raise ValueError("input CSV must have an x,y header")
        rows: list[dict[str, float]] = []
        for row_number, raw in enumerate(reader, start=2):
            if not any(value and value.strip() for value in raw.values()):
                continue
            point = {
                "x": finite_number(raw["x"], "x", row_number),
                "y": finite_number(raw["y"], "y", row_number),
            }
            for field in ("width_left", "width_right", "v_ref"):
                value = raw.get(field, "")
                point[field] = (
                    finite_number(value, field, row_number)
                    if value is not None and value.strip()
                    else defaults[field]
                )
            if point["width_left"] <= 0.0 or point["width_right"] <= 0.0:
                raise ValueError(f"row {row_number}: track widths must be positive")
            if point["v_ref"] <= 0.0:
                raise ValueError(f"row {row_number}: v_ref must be positive")
            if not rows or math.hypot(
                point["x"] - rows[-1]["x"], point["y"] - rows[-1]["y"]
            ) > 1.0e-6:
                rows.append(point)
    if len(rows) > 2 and math.hypot(
        rows[0]["x"] - rows[-1]["x"], rows[0]["y"] - rows[-1]["y"]
    ) <= 1.0e-6:
        rows.pop()
    if len(rows) < 3:
        raise ValueError("closed centerline needs at least three distinct points")
    return rows


def interpolate(a: float, b: float, ratio: float) -> float:
    return a + ratio * (b - a)


def resample(points: list[dict[str, float]], spacing: float) -> list[dict[str, float]]:
    segment_lengths = []
    cumulative = [0.0]
    for index, point in enumerate(points):
        following = points[(index + 1) % len(points)]
        length = math.hypot(following["x"] - point["x"], following["y"] - point["y"])
        if length <= 1.0e-6:
            raise ValueError(f"zero-length centerline segment at point {index}")
        segment_lengths.append(length)
        cumulative.append(cumulative[-1] + length)
    total_length = cumulative[-1]
    count = max(3, int(math.ceil(total_length / spacing)))
    actual_spacing = total_length / count
    samples: list[dict[str, float]] = []
    segment = 0
    for sample_index in range(count):
        distance = sample_index * actual_spacing
        while segment + 1 < len(cumulative) and cumulative[segment + 1] <= distance:
            segment += 1
        ratio = (distance - cumulative[segment]) / segment_lengths[segment]
        current = points[segment]
        following = points[(segment + 1) % len(points)]
        samples.append({
            "s": distance,
            "x": interpolate(current["x"], following["x"], ratio),
            "y": interpolate(current["y"], following["y"], ratio),
            "width_left": interpolate(current["width_left"], following["width_left"], ratio),
            "width_right": interpolate(current["width_right"], following["width_right"], ratio),
            "input_v_ref": interpolate(current["v_ref"], following["v_ref"], ratio),
        })

    for index, sample in enumerate(samples):
        previous = samples[(index - 1) % count]
        following = samples[(index + 1) % count]
        sample["yaw"] = math.atan2(
            following["y"] - previous["y"], following["x"] - previous["x"]
        )
    for index, sample in enumerate(samples):
        previous_yaw = samples[(index - 1) % count]["yaw"]
        following_yaw = samples[(index + 1) % count]["yaw"]
        delta_yaw = math.atan2(
            math.sin(following_yaw - previous_yaw), math.cos(following_yaw - previous_yaw)
        )
        sample["curvature"] = delta_yaw / (2.0 * actual_spacing)
    return samples


def apply_speed_limit(
    samples: list[dict[str, float]], max_speed: float, min_speed: float,
    max_lateral_acceleration: float,
) -> None:
    hard_limits = []
    for sample in samples:
        curvature_speed = math.sqrt(
            max_lateral_acceleration / max(abs(sample["curvature"]), 1.0e-4)
        )
        # min_speed is a preferred floor, never an excuse to violate the
        # curvature-derived hard limit.
        preferred_speed = max(
            min_speed, min(sample["input_v_ref"], max_speed)
        )
        sample["v_ref"] = min(preferred_speed, curvature_speed)
        hard_limits.append(sample["v_ref"])
    # A circular moving-average may lower neighbouring straight-line speeds,
    # but must never raise a point above its original lateral-acceleration cap.
    for _ in range(3):
        speeds = [sample["v_ref"] for sample in samples]
        for index, sample in enumerate(samples):
            smoothed = sum(
                speeds[(index + offset) % len(samples)]
                for offset in (-2, -1, 0, 1, 2)
            ) / 5.0
            sample["v_ref"] = min(hard_limits[index], smoothed)


def validate_output(samples: list[dict[str, float]]) -> None:
    previous_s = -math.inf
    for index, sample in enumerate(samples):
        for field in OUTPUT_FIELDS:
            if field not in sample or not math.isfinite(sample[field]):
                raise ValueError(f"generated sample {index}: invalid {field}")
        if sample["s"] <= previous_s:
            raise ValueError("generated s values are not strictly increasing")
        if sample["v_ref"] <= 0.0 or sample["width_left"] <= 0.0 or sample["width_right"] <= 0.0:
            raise ValueError(f"generated sample {index}: speed and widths must be positive")
        previous_s = sample["s"]


def write_output(path: Path, samples: list[dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as destination:
        writer = csv.DictWriter(destination, fieldnames=OUTPUT_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for sample in samples:
            writer.writerow({field: f"{sample[field]:.9f}" for field in OUTPUT_FIELDS})


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="input CSV with at least x,y columns")
    parser.add_argument("output", type=Path, help="output strict race-line CSV")
    parser.add_argument("--spacing", type=float, default=0.05)
    parser.add_argument("--width-left", type=float, default=0.75)
    parser.add_argument("--width-right", type=float, default=0.75)
    parser.add_argument("--max-speed", type=float, default=2.0)
    parser.add_argument("--min-speed", type=float, default=0.3)
    parser.add_argument("--max-lateral-acceleration", type=float, default=3.0)
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    numeric_arguments = (
        arguments.spacing, arguments.width_left, arguments.width_right,
        arguments.max_speed, arguments.min_speed, arguments.max_lateral_acceleration,
    )
    if not all(math.isfinite(value) and value > 0.0 for value in numeric_arguments):
        raise ValueError("spacing, widths, speeds and lateral acceleration must be positive")
    if arguments.min_speed > arguments.max_speed:
        raise ValueError("min-speed cannot exceed max-speed")
    defaults = {
        "width_left": arguments.width_left,
        "width_right": arguments.width_right,
        "v_ref": arguments.max_speed,
    }
    points = read_centerline(arguments.input, defaults)
    samples = resample(points, arguments.spacing)
    apply_speed_limit(
        samples, arguments.max_speed, arguments.min_speed,
        arguments.max_lateral_acceleration,
    )
    validate_output(samples)
    write_output(arguments.output, samples)
    print(f"wrote {len(samples)} closed-loop waypoints to {arguments.output}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError) as error:
        print(f"generate_raceline: {error}", file=sys.stderr)
        sys.exit(2)
