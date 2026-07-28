#!/usr/bin/env python3

"""Convert semicolon optimizer output into the RoboRacer race-line CSV."""

import argparse
import csv
import math
from pathlib import Path


OUTPUT_FIELDS = (
    "s",
    "x",
    "y",
    "yaw",
    "curvature",
    "v_ref",
    "width_left",
    "width_right",
)


def load_optimizer_output(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open(encoding="utf-8") as source:
        for line_number, line in enumerate(source, start=1):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            fields = stripped.split(";")
            values = [float(field) for field in fields]
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"{path}:{line_number}: non-finite value")
            if len(values) == 9:
                # Legacy optimizer output:
                # s;x;y;psi;curvature;speed;acceleration;width_left;width_right
                rows.append(
                    {
                        "s": values[0],
                        "x": values[1],
                        "y": values[2],
                        # Legacy psi uses the optimizer's normal-angle
                        # convention; derive the tangent yaw geometrically.
                        "source_yaw": math.nan,
                        "source_curvature": values[4],
                        "source_speed": values[5],
                        "width_left": values[7],
                        "width_right": values[8],
                    }
                )
            elif len(values) == 7:
                # trajectory_planning_helpers output:
                # x;y;speed;trajectory_direction;s;width_left;width_right
                rows.append(
                    {
                        "s": values[4],
                        "x": values[0],
                        "y": values[1],
                        "source_yaw": values[3],
                        "source_curvature": math.nan,
                        "source_speed": values[2],
                        "width_left": values[5],
                        "width_right": values[6],
                    }
                )
            else:
                raise ValueError(
                    f"{path}:{line_number}: expected 7 or 9 fields, "
                    f"got {len(values)}"
                )
    if len(rows) < 3:
        raise ValueError("race line needs at least three samples")
    return rows


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def geometry(
    previous: dict[str, float],
    current: dict[str, float],
    following: dict[str, float],
) -> tuple[float, float]:
    chord_x = following["x"] - previous["x"]
    chord_y = following["y"] - previous["y"]
    yaw = math.atan2(chord_y, chord_x)

    first_x = current["x"] - previous["x"]
    first_y = current["y"] - previous["y"]
    second_x = following["x"] - current["x"]
    second_y = following["y"] - current["y"]
    denominator = (
        math.hypot(first_x, first_y)
        * math.hypot(second_x, second_y)
        * math.hypot(chord_x, chord_y)
    )
    curvature = 0.0
    if denominator > 1.0e-12:
        curvature = 2.0 * (first_x * second_y - first_y * second_x) / denominator
    return normalize_angle(yaw), curvature


def convert(
    rows: list[dict[str, float]],
    max_speed: float,
    max_lateral_acceleration: float,
) -> list[dict[str, float]]:
    output: list[dict[str, float]] = []
    for index, row in enumerate(rows):
        previous = rows[(index - 1) % len(rows)]
        following = rows[(index + 1) % len(rows)]
        geometry_yaw, curvature = geometry(previous, row, following)
        source_yaw = row["source_yaw"]
        yaw = geometry_yaw
        if math.isfinite(source_yaw):
            source_yaw = normalize_angle(source_yaw)
            yaw_difference = abs(normalize_angle(source_yaw - geometry_yaw))
            if yaw_difference > 0.15:
                raise ValueError(
                    f"heading convention mismatch at sample {index}: "
                    f"{source_yaw} versus {geometry_yaw}"
                )
            yaw = source_yaw
        source_curvature = row["source_curvature"]
        curvature_difference = abs(curvature - source_curvature)
        if math.isfinite(source_curvature) and curvature_difference > 0.05:
            raise ValueError(
                f"curvature convention mismatch at sample {index}: "
                f"{curvature} versus {source_curvature}"
            )
        curvature_speed = (
            math.sqrt(max_lateral_acceleration / abs(curvature))
            if abs(curvature) > 1.0e-9
            else max_speed
        )
        output.append(
            {
                "s": row["s"],
                "x": row["x"],
                "y": row["y"],
                "yaw": yaw,
                "curvature": curvature,
                "v_ref": min(row["source_speed"], max_speed, curvature_speed),
                "width_left": row["width_left"],
                "width_right": row["width_right"],
            }
        )
    return output


def validate(rows: list[dict[str, float]]) -> None:
    previous_s = -math.inf
    for index, row in enumerate(rows):
        if not all(math.isfinite(row[field]) for field in OUTPUT_FIELDS):
            raise ValueError(f"non-finite output at sample {index}")
        if row["s"] <= previous_s:
            raise ValueError("s must be strictly increasing")
        if row["v_ref"] <= 0.0 or row["width_left"] <= 0.0 or row["width_right"] <= 0.0:
            raise ValueError(f"invalid speed or width at sample {index}")
        previous_s = row["s"]
    closure = math.hypot(
        rows[-1]["x"] - rows[0]["x"],
        rows[-1]["y"] - rows[0]["y"],
    )
    if closure <= 0.0 or closure > 0.30:
        raise ValueError(f"race line is not closed: closure={closure:.3f} m")


def write_output(path: Path, rows: list[dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as destination:
        writer = csv.DictWriter(destination, fieldnames=OUTPUT_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: f"{row[field]:.9f}" for field in OUTPUT_FIELDS})


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--max-speed", type=float, default=4.0)
    parser.add_argument("--max-lateral-acceleration", type=float, default=4.0)
    arguments = parser.parse_args()
    if arguments.max_speed <= 0.0 or arguments.max_lateral_acceleration <= 0.0:
        raise ValueError("speed and lateral acceleration limits must be positive")

    rows = convert(
        load_optimizer_output(arguments.input),
        arguments.max_speed,
        arguments.max_lateral_acceleration,
    )
    validate(rows)
    write_output(arguments.output, rows)
    print(f"wrote {len(rows)} points to {arguments.output}")


if __name__ == "__main__":
    main()
