#!/usr/bin/env python3
"""Extract a closed MPPI race line from an annular ROS occupancy-grid map."""

import argparse
import math
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

from generate_raceline import apply_speed_limit, resample, validate_output, write_output


def load_map(map_yaml: Path):
    with map_yaml.open(encoding="utf-8") as source:
        metadata = yaml.safe_load(source)
    required = {"image", "resolution", "origin", "negate", "free_thresh"}
    if not isinstance(metadata, dict) or not required.issubset(metadata):
        raise ValueError("map YAML is missing required ROS map fields")

    image_path = Path(str(metadata["image"]))
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f"cannot read map image: {image_path}")

    resolution = float(metadata["resolution"])
    origin = [float(value) for value in metadata["origin"]]
    free_threshold = float(metadata["free_thresh"])
    if resolution <= 0.0 or len(origin) != 3:
        raise ValueError("map resolution and origin are invalid")

    normalized = image.astype(np.float32) / 255.0
    occupancy = normalized if int(metadata["negate"]) else 1.0 - normalized
    free_mask = (occupancy < free_threshold).astype(np.uint8)
    return image, free_mask, resolution, origin, image_path


def world_to_pixel(x, y, height, resolution, origin):
    cosine = math.cos(origin[2])
    sine = math.sin(origin[2])
    dx = x - origin[0]
    dy = y - origin[1]
    map_x = cosine * dx + sine * dy
    map_y = -sine * dx + cosine * dy
    column = int(round(map_x / resolution - 0.5))
    row = int(round(height - map_y / resolution - 0.5))
    return row, column


def pixel_to_world(points, height, resolution, origin):
    map_x = (points[:, 0] + 0.5) * resolution
    map_y = (height - points[:, 1] - 0.5) * resolution
    cosine = math.cos(origin[2])
    sine = math.sin(origin[2])
    world_x = origin[0] + cosine * map_x - sine * map_y
    world_y = origin[1] + sine * map_x + cosine * map_y
    return np.column_stack((world_x, world_y))


def component_contours(labels, label):
    mask = np.where(labels == label, 255, 0).astype(np.uint8)
    contours, hierarchy = cv2.findContours(
        mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE
    )
    return mask, contours, hierarchy


def choose_track_component(free_mask, seed_x, seed_y, resolution, origin):
    count, labels, stats, _ = cv2.connectedComponentsWithStats(free_mask, 8)
    if seed_x is not None:
        row, column = world_to_pixel(
            seed_x, seed_y, free_mask.shape[0], resolution, origin
        )
        if not (0 <= row < free_mask.shape[0] and 0 <= column < free_mask.shape[1]):
            raise ValueError("seed point is outside the map")
        label = int(labels[row, column])
        if label == 0:
            raise ValueError("seed point is occupied; choose a point on the track")
        mask, contours, hierarchy = component_contours(labels, label)
        if len(contours) < 2:
            raise ValueError("seed component is not an annular track corridor")
        return mask, contours, hierarchy

    candidates = []
    for label in range(1, count):
        left = int(stats[label, cv2.CC_STAT_LEFT])
        top = int(stats[label, cv2.CC_STAT_TOP])
        width = int(stats[label, cv2.CC_STAT_WIDTH])
        height = int(stats[label, cv2.CC_STAT_HEIGHT])
        touches_edge = (
            left == 0 or top == 0 or left + width == free_mask.shape[1]
            or top + height == free_mask.shape[0]
        )
        if touches_edge:
            continue
        mask, contours, hierarchy = component_contours(labels, label)
        if hierarchy is None or len(contours) < 2:
            continue
        has_hole = any(int(entry[3]) >= 0 for entry in hierarchy[0])
        if has_hole:
            candidates.append((int(stats[label, cv2.CC_STAT_AREA]), mask, contours, hierarchy))
    if not candidates:
        raise ValueError("no enclosed annular free-space component found; provide --seed-x/--seed-y")
    candidates.sort(key=lambda item: item[0], reverse=True)
    return candidates[0][1:]


def circular_smooth(points, sigma):
    if sigma <= 0.0:
        return points.copy()
    radius = max(1, int(math.ceil(3.0 * sigma)))
    offsets = np.arange(-radius, radius + 1, dtype=np.float64)
    kernel = np.exp(-0.5 * (offsets / sigma) ** 2)
    kernel /= np.sum(kernel)
    smoothed = np.zeros_like(points, dtype=np.float64)
    for offset, weight in zip(offsets.astype(int), kernel):
        smoothed += weight * np.roll(points, offset, axis=0)
    return smoothed


def extract_midline(mask, contours, smoothing_sigma):
    ordered = sorted(contours, key=lambda contour: abs(cv2.contourArea(contour)), reverse=True)
    if len(ordered) < 2:
        raise ValueError("track component needs distinct inner and outer boundaries")
    outer = ordered[0][:, 0, :].astype(np.float32)
    inner = ordered[1][:, 0, :].astype(np.float32)
    if len(outer) < 20 or len(inner) < 20:
        raise ValueError("track boundaries are too short")

    index = cv2.flann_Index(inner, {"algorithm": 1, "trees": 8})
    nearest, _ = index.knnSearch(outer, 1, params={"checks": 256})
    midline = 0.5 * (outer + inner[nearest[:, 0]])
    midline = circular_smooth(midline, smoothing_sigma)

    rows = np.clip(np.rint(midline[:, 1]).astype(int), 0, mask.shape[0] - 1)
    columns = np.clip(np.rint(midline[:, 0]).astype(int), 0, mask.shape[1] - 1)
    if np.mean(mask[rows, columns] > 0) < 0.999:
        raise ValueError("extracted midline leaves the free-space track component")
    clearance_pixels = cv2.distanceTransform(mask, cv2.DIST_L2, 5)[rows, columns]
    return midline, clearance_pixels


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map", type=Path, help="ROS map YAML")
    parser.add_argument("output", type=Path, help="output strict MPPI race-line CSV")
    parser.add_argument("--seed-x", type=float)
    parser.add_argument("--seed-y", type=float)
    parser.add_argument("--spacing", type=float, default=0.05)
    parser.add_argument("--max-speed", type=float, default=2.0)
    parser.add_argument("--min-speed", type=float, default=0.3)
    parser.add_argument("--max-lateral-acceleration", type=float, default=3.0)
    parser.add_argument("--smoothing-sigma", type=float, default=4.0)
    parser.add_argument("--width-scale", type=float, default=0.95)
    parser.add_argument("--min-clearance", type=float, default=0.20)
    parser.add_argument("--reverse", action="store_true")
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    if (arguments.seed_x is None) != (arguments.seed_y is None):
        raise ValueError("--seed-x and --seed-y must be provided together")
    positive = (
        arguments.spacing, arguments.max_speed, arguments.min_speed,
        arguments.max_lateral_acceleration, arguments.width_scale,
        arguments.min_clearance,
    )
    if not all(math.isfinite(value) and value > 0.0 for value in positive):
        raise ValueError("spacing, speed, acceleration, width scale and clearance must be positive")
    if arguments.min_speed > arguments.max_speed or arguments.width_scale > 1.0:
        raise ValueError("min-speed must not exceed max-speed and width-scale must be <= 1")

    _, free_mask, resolution, origin, image_path = load_map(arguments.map)
    mask, contours, _ = choose_track_component(
        free_mask, arguments.seed_x, arguments.seed_y, resolution, origin
    )
    midline_pixels, clearance_pixels = extract_midline(
        mask, contours, arguments.smoothing_sigma
    )
    if arguments.reverse:
        midline_pixels = midline_pixels[::-1]
        clearance_pixels = clearance_pixels[::-1]

    minimum_clearance = float(np.min(clearance_pixels)) * resolution
    if minimum_clearance < arguments.min_clearance:
        raise ValueError(
            f"minimum extracted clearance {minimum_clearance:.3f} m is below "
            f"{arguments.min_clearance:.3f} m"
        )

    world = pixel_to_world(midline_pixels, mask.shape[0], resolution, origin)
    raw_points = []
    for point, clearance_pixel in zip(world, clearance_pixels):
        width = max(
            arguments.min_clearance,
            float(clearance_pixel) * resolution * arguments.width_scale,
        )
        raw_points.append({
            "x": float(point[0]),
            "y": float(point[1]),
            "width_left": width,
            "width_right": width,
            "v_ref": arguments.max_speed,
        })

    samples = resample(raw_points, arguments.spacing)
    apply_speed_limit(
        samples, arguments.max_speed, arguments.min_speed,
        arguments.max_lateral_acceleration,
    )
    validate_output(samples)
    write_output(arguments.output, samples)
    print(
        f"wrote {len(samples)} waypoints to {arguments.output} from {image_path}; "
        f"minimum center clearance={minimum_clearance:.3f} m"
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, cv2.error, yaml.YAMLError) as error:
        print(f"extract_raceline_from_map: {error}", file=sys.stderr)
        sys.exit(2)
