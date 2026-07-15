"""Adaptive Follow-the-Gap controller for F1TENTH.

The planner is deliberately independent from ROS so its scan processing can be
tested and tuned without starting a node.  ``FTGNode`` only handles parameters,
message I/O, and rate limiting of the resulting commands.
"""

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan


@dataclass(frozen=True)
class FTGConfig:
    """Parameters used by the ROS-independent gap planner."""

    field_of_view_deg: float = 180.0
    max_lidar_range: float = 10.0
    min_lidar_range: float = 0.05
    smoothing_window: int = 5
    disparity_threshold: float = 0.40
    safety_radius: float = 0.35
    min_clearance: float = 0.45
    min_gap_width: float = 0.65
    best_point_window: int = 9
    distance_weight: float = 1.0
    heading_weight: float = 0.35
    gap_center_weight: float = 0.25


class FollowTheGapPlanner:
    """Turn a laser scan into a safe target bearing."""

    def __init__(self, config: FTGConfig):
        self.config = config

    @staticmethod
    def _moving_average(ranges: np.ndarray, window: int) -> np.ndarray:
        """Smooth noise without ever making an observed obstacle look farther."""
        if window <= 1 or ranges.size < 2:
            return ranges.copy()

        window = min(window, ranges.size)
        if window % 2 == 0:
            window -= 1
        if window <= 1:
            return ranges.copy()

        half = window // 2
        padded = np.pad(ranges, (half, half), mode="edge")
        kernel = np.full(window, 1.0 / window, dtype=float)
        averaged = np.convolve(padded, kernel, mode="valid")

        # A conventional mean can erase a small close obstacle.  Taking the
        # minimum keeps that return and only spreads its influence slightly.
        return np.where(ranges > 0.0, np.minimum(ranges, averaged), 0.0)

    def preprocess_scan(
        self,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        sensor_range_min: float,
        sensor_range_max: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Sanitize, crop, and conservatively smooth a LaserScan."""
        raw = np.asarray(ranges, dtype=float)
        if raw.size == 0 or not math.isfinite(angle_increment) or angle_increment <= 0.0:
            return np.empty(0, dtype=float), np.empty(0, dtype=float)

        finite_sensor_max = (
            sensor_range_max
            if math.isfinite(sensor_range_max) and sensor_range_max > 0.0
            else self.config.max_lidar_range
        )
        range_cap = min(self.config.max_lidar_range, finite_sensor_max)
        finite_sensor_min = (
            sensor_range_min
            if math.isfinite(sensor_range_min) and sensor_range_min > 0.0
            else self.config.min_lidar_range
        )
        range_floor = max(self.config.min_lidar_range, finite_sensor_min)
        if range_floor >= range_cap:
            return np.empty(0, dtype=float), np.empty(0, dtype=float)

        clean = np.nan_to_num(
            raw,
            nan=0.0,
            posinf=range_cap,
            neginf=0.0,
        )
        clean = np.where(clean >= range_floor, np.minimum(clean, range_cap), 0.0)

        angles = angle_min + np.arange(raw.size, dtype=float) * angle_increment
        half_fov = math.radians(self.config.field_of_view_deg) * 0.5
        front_mask = np.abs(angles) <= half_fov
        clean = clean[front_mask]
        angles = angles[front_mask]

        clean = self._moving_average(clean, self.config.smoothing_window)
        return clean, angles

    def create_safe_zone(self, ranges: np.ndarray, angle_increment: float) -> np.ndarray:
        """Inflate corners and the nearest obstacle by the vehicle safety radius."""
        safe = ranges.copy()
        if safe.size == 0:
            return safe

        # Disparity extension prevents the target from cutting behind the near
        # edge of a wall.  Work from the original scan so extensions do not
        # cascade across the complete field of view.
        source = ranges.copy()
        disparities = np.flatnonzero(
            np.abs(np.diff(source)) >= self.config.disparity_threshold
        )
        for left_index in disparities:
            right_index = left_index + 1
            left_distance = source[left_index]
            right_distance = source[right_index]

            if left_distance <= 0.0 or right_distance <= 0.0:
                continue
            if left_distance < right_distance:
                self._extend_obstacle(
                    safe,
                    close_index=left_index,
                    close_distance=left_distance,
                    direction=1,
                    angle_increment=angle_increment,
                )
            elif right_distance < left_distance:
                self._extend_obstacle(
                    safe,
                    close_index=right_index,
                    close_distance=right_distance,
                    direction=-1,
                    angle_increment=angle_increment,
                )

        valid_indices = np.flatnonzero(safe > 0.0)
        if valid_indices.size == 0:
            return safe

        closest_index = valid_indices[np.argmin(safe[valid_indices])]
        closest_distance = safe[closest_index]
        bubble_angle = math.atan2(self.config.safety_radius, closest_distance)
        bubble_points = max(1, math.ceil(bubble_angle / angle_increment))
        start = max(0, closest_index - bubble_points)
        end = min(safe.size, closest_index + bubble_points + 1)
        safe[start:end] = 0.0
        return safe

    def _extend_obstacle(
        self,
        ranges: np.ndarray,
        close_index: int,
        close_distance: float,
        direction: int,
        angle_increment: float,
    ) -> None:
        """Extend the close side of one depth discontinuity into its far side."""
        half_angle = math.atan2(self.config.safety_radius, close_distance)
        point_count = max(1, math.ceil(half_angle / angle_increment))

        for offset in range(1, point_count + 1):
            index = close_index + direction * offset
            if index < 0 or index >= ranges.size or ranges[index] <= 0.0:
                break
            ranges[index] = min(ranges[index], close_distance)

    @staticmethod
    def _contiguous_regions(mask: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Return inclusive starts and exclusive ends of all true regions."""
        edges = np.diff(np.pad(mask.astype(np.int8), (1, 1)))
        return np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)

    def find_target(
        self,
        ranges: np.ndarray,
        angles: np.ndarray,
        angle_increment: float,
    ) -> Optional[Tuple[float, float]]:
        """Return ``(target bearing, target clearance)`` for the best safe gap."""
        if ranges.size == 0 or ranges.size != angles.size:
            return None

        free_mask = ranges >= self.config.min_clearance
        starts, ends = self._contiguous_regions(free_mask)
        if starts.size == 0:
            return None

        gap_candidates = []
        for start, end in zip(starts, ends):
            width_points = int(end - start)
            if width_points <= 0:
                continue

            gap_ranges = ranges[start:end]
            gap_angles = angles[start:end]
            width_angle = width_points * angle_increment
            representative_distance = float(np.percentile(gap_ranges, 50.0))
            physical_width = 2.0 * representative_distance * math.sin(
                min(width_angle, math.pi) * 0.5
            )
            mean_range = float(np.mean(gap_ranges))
            center_angle = float((gap_angles[0] + gap_angles[-1]) * 0.5)
            score = (
                width_angle * (0.5 + mean_range / self.config.max_lidar_range)
                + 0.35 * math.cos(center_angle)
                + 0.20 * float(np.max(gap_ranges)) / self.config.max_lidar_range
            )
            candidate = (score, int(start), int(end))
            if physical_width >= self.config.min_gap_width:
                gap_candidates.append(candidate)

        # Never command motion through a gap that is narrower than the configured
        # vehicle envelope.  Stopping is safer than a best-effort narrow fallback.
        if not gap_candidates:
            return None

        _, start, end = max(gap_candidates, key=lambda candidate: candidate[0])
        gap_ranges = ranges[start:end]
        gap_angles = angles[start:end]
        count = end - start

        # Average local depth before scoring so a single max-range ray cannot
        # pull the car toward a noisy reflection.
        window = min(self.config.best_point_window, count)
        if window % 2 == 0:
            window -= 1
        if window > 1:
            kernel = np.full(window, 1.0 / window, dtype=float)
            half = window // 2
            padded = np.pad(gap_ranges, (half, half), mode="edge")
            local_depth = np.convolve(padded, kernel, mode="valid")
        else:
            local_depth = gap_ranges

        edge_distance = np.minimum(np.arange(count) + 1, np.arange(count, 0, -1))
        edge_score = edge_distance / max(float(np.max(edge_distance)), 1.0)
        scores = (
            self.config.distance_weight
            * np.clip(local_depth / self.config.max_lidar_range, 0.0, 1.0)
            + self.config.heading_weight * np.cos(gap_angles)
            + self.config.gap_center_weight * edge_score
        )
        target_offset = int(np.argmax(scores))
        return float(gap_angles[target_offset]), float(gap_ranges[target_offset])


class FTGNode(Node):
    """ROS 2 adapter for the adaptive Follow-the-Gap planner."""

    def __init__(self):
        super().__init__("ftg_node")

        scan_topic = self._parameter("scan_topic", "/scan")
        drive_topic = self._parameter("drive_topic", "/ackermann_cmd")

        planner_config = FTGConfig(
            field_of_view_deg=self._positive_parameter("field_of_view_deg", 180.0),
            max_lidar_range=self._positive_parameter("max_lidar_range", 10.0),
            min_lidar_range=self._positive_parameter("min_lidar_range", 0.05),
            smoothing_window=max(
                1, int(round(self._positive_parameter("smoothing_window", 5)))
            ),
            disparity_threshold=self._positive_parameter("disparity_threshold", 0.40),
            safety_radius=self._positive_parameter("safety_radius", 0.35),
            min_clearance=self._positive_parameter("min_clearance", 0.45),
            min_gap_width=self._positive_parameter("min_gap_width", 0.65),
            best_point_window=max(
                1, int(round(self._positive_parameter("best_point_window", 9)))
            ),
            distance_weight=self._positive_parameter("distance_weight", 1.0),
            heading_weight=self._positive_parameter("heading_weight", 0.35),
            gap_center_weight=self._positive_parameter("gap_center_weight", 0.25),
        )
        self.planner = FollowTheGapPlanner(planner_config)

        self.max_steering_angle = self._positive_parameter("max_steering_angle", 0.42)
        self.steering_gain = self._positive_parameter("steering_gain", 1.0)
        self.steering_smoothing = self._bounded_parameter(
            "steering_smoothing", 0.35, 0.0, 1.0
        )
        self.max_steering_rate = self._positive_parameter("max_steering_rate", 1.5)
        self.min_speed = self._positive_parameter("min_speed", 1.0)
        self.max_speed = self._positive_parameter("max_speed", 4.0)
        if self.min_speed > self.max_speed:
            self.get_logger().warning("min_speed exceeds max_speed; clamping it")
            self.min_speed = self.max_speed
        self.emergency_distance = self._positive_parameter("emergency_distance", 0.45)
        self.slow_distance = self._positive_parameter("slow_distance", 2.5)
        if self.slow_distance <= self.emergency_distance:
            self.get_logger().warning(
                "slow_distance must exceed emergency_distance; using safe default"
            )
            self.slow_distance = max(2.5, self.emergency_distance + 0.5)
        self.front_sector_deg = self._positive_parameter("front_sector_deg", 20.0)
        self.wheelbase = self._positive_parameter("wheelbase", 0.33)
        self.max_lateral_accel = self._positive_parameter("max_lateral_accel", 4.0)
        self.max_accel = self._positive_parameter("max_accel", 2.0)
        self.max_decel = self._positive_parameter("max_decel", 5.0)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscriber = self.create_subscription(
            LaserScan, str(scan_topic), self.scan_callback, qos
        )
        self.publisher = self.create_publisher(AckermannDriveStamped, str(drive_topic), 10)

        self.current_steering_angle = 0.0
        self.current_speed = 0.0
        self.last_update_ns: Optional[int] = None
        self.scan_count = 0
        self.get_logger().info(
            f"FTG started: {scan_topic} -> {drive_topic}, "
            f"speed={self.min_speed:.1f}..{self.max_speed:.1f} m/s"
        )

    def _parameter(self, name: str, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _positive_parameter(self, name: str, default: float) -> float:
        value = float(self._parameter(name, default))
        if not math.isfinite(value) or value <= 0.0:
            self.get_logger().warning(f"Invalid {name}={value}; using {default}")
            return float(default)
        return value

    def _bounded_parameter(
        self, name: str, default: float, lower: float, upper: float
    ) -> float:
        value = float(self._parameter(name, default))
        if not math.isfinite(value):
            self.get_logger().warning(f"Invalid {name}={value}; using {default}")
            return default
        return float(np.clip(value, lower, upper))

    def _elapsed_time(self) -> float:
        now_ns = self.get_clock().now().nanoseconds
        if self.last_update_ns is None or now_ns <= self.last_update_ns:
            elapsed = 0.05
        else:
            elapsed = (now_ns - self.last_update_ns) * 1e-9
        self.last_update_ns = now_ns
        return float(np.clip(elapsed, 0.01, 0.20))

    def _front_clearance(self, ranges: np.ndarray, angles: np.ndarray) -> float:
        half_sector = math.radians(self.front_sector_deg) * 0.5
        front = ranges[np.abs(angles) <= half_sector]
        valid_front = front[front > 0.0]
        if valid_front.size == 0:
            return 0.0

        # A low percentile rejects a lone noisy speck while remaining much more
        # conservative than a mean or a maximum.
        return float(np.percentile(valid_front, 10.0))

    def _steering_command(self, target_angle: float, elapsed: float) -> float:
        requested = float(
            np.clip(
                self.steering_gain * target_angle,
                -self.max_steering_angle,
                self.max_steering_angle,
            )
        )
        filtered = (
            self.steering_smoothing * requested
            + (1.0 - self.steering_smoothing) * self.current_steering_angle
        )
        maximum_change = self.max_steering_rate * elapsed
        command = float(
            np.clip(
                filtered,
                self.current_steering_angle - maximum_change,
                self.current_steering_angle + maximum_change,
            )
        )
        self.current_steering_angle = command
        return command

    def _speed_command(self, steering: float, clearance: float, elapsed: float) -> float:
        if clearance <= self.emergency_distance:
            self.current_speed = 0.0
            return 0.0

        steering_ratio = min(abs(steering) / self.max_steering_angle, 1.0)
        corner_speed = self.max_speed - (
            self.max_speed - self.min_speed
        ) * steering_ratio**1.5

        # Respect an approximate lateral-acceleration limit for the commanded
        # curvature, independently of the heuristic corner-speed curve.
        tangent = abs(math.tan(steering))
        if tangent > 1e-4:
            lateral_speed = math.sqrt(
                self.max_lateral_accel * self.wheelbase / tangent
            )
            corner_speed = min(corner_speed, lateral_speed)

        clearance_span = max(self.slow_distance - self.emergency_distance, 1e-3)
        clearance_ratio = float(
            np.clip(
                (clearance - self.emergency_distance) / clearance_span,
                0.0,
                1.0,
            )
        )
        clearance_speed = self.min_speed + clearance_ratio * (
            self.max_speed - self.min_speed
        )
        stopping_speed = math.sqrt(
            2.0 * self.max_decel * max(clearance - self.emergency_distance, 0.0)
        )
        requested = min(corner_speed, clearance_speed, stopping_speed, self.max_speed)

        lower = max(0.0, self.current_speed - self.max_decel * elapsed)
        upper = min(self.max_speed, self.current_speed + self.max_accel * elapsed)
        self.current_speed = float(np.clip(requested, lower, upper))
        return self.current_speed

    def _publish_stop(self, source_header=None) -> None:
        self.current_speed = 0.0
        command = AckermannDriveStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        if source_header is not None:
            command.header.frame_id = source_header.frame_id
        command.drive.steering_angle = float(self.current_steering_angle)
        command.drive.speed = 0.0
        self.publisher.publish(command)

    def scan_callback(self, msg: LaserScan) -> None:
        self.scan_count += 1
        elapsed = self._elapsed_time()
        clean_ranges, angles = self.planner.preprocess_scan(
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            msg.range_min,
            msg.range_max,
        )
        if clean_ranges.size == 0:
            self._publish_stop(msg.header)
            if self.scan_count % 20 == 1:
                self.get_logger().warning("Invalid or empty LaserScan; stopping")
            return

        safe_ranges = self.planner.create_safe_zone(clean_ranges, msg.angle_increment)
        target = self.planner.find_target(safe_ranges, angles, msg.angle_increment)
        if target is None:
            self._publish_stop(msg.header)
            if self.scan_count % 20 == 1:
                self.get_logger().warning("No traversable gap; stopping")
            return

        target_angle, target_clearance = target
        front_clearance = self._front_clearance(clean_ranges, angles)
        path_clearance = min(front_clearance, target_clearance)
        steering = self._steering_command(target_angle, elapsed)
        speed = self._speed_command(steering, path_clearance, elapsed)

        command = AckermannDriveStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = msg.header.frame_id
        command.drive.steering_angle = steering
        command.drive.speed = speed
        self.publisher.publish(command)

        if self.scan_count % 20 == 0:
            self.get_logger().info(
                f"FTG: target={math.degrees(target_angle):+.1f} deg, "
                f"steer={steering:+.3f} rad, speed={speed:.2f} m/s, "
                f"front={front_clearance:.2f} m"
            )


def main(args=None):
    rclpy.init(args=args)
    node = FTGNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
