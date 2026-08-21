#!/usr/bin/env python3
"""Calls /start_trajectory with an explicit initial pose, then exits.

Replaces an ExecuteProcess(["ros2", "service", "call", ...]) in the launch file.
Shelling out was unreliable: `ros2` is not guaranteed to be on the launch
subprocess PATH, a non-zero exit is not surfaced anywhere, and there is no way
to wait properly for the service or to retry. This node waits for the service,
retries, and logs a specific reason on failure.

Needed because my_car_localization.lua sets POSE_GRAPH.global_sampling_ratio=0,
so Cartographer will not search the map to find itself and must be told where
it starts. Note that TRAJECTORY_BUILDER.initial_trajectory_pose in the .lua does
NOT work for this -- that key is only read on this service path, and left in the
.lua it is never consumed and Cartographer aborts with
"Key 'initial_trajectory_pose' was used the wrong number of times."

Standalone use:
    ros2 run robo_cartographer start_trajectory.py \\
        --ros-args -p start_x:=-3.80 -p start_y:=-0.16 -p start_yaw:=-0.133
"""

import math
import sys

import rclpy
from rclpy.node import Node

from cartographer_ros_msgs.srv import StartTrajectory, GetTrajectoryStates


class TrajectoryStarter(Node):
    def __init__(self):
        super().__init__("start_trajectory_client")

        self.declare_parameter("configuration_directory", "")
        self.declare_parameter("configuration_basename",
                               "my_car_localization.lua")
        self.declare_parameter("start_x", -3.80)
        self.declare_parameter("start_y", -0.16)
        self.declare_parameter("start_yaw", -0.133)
        self.declare_parameter("relative_to_trajectory_id", 0)
        # Loading a large .pbstream can take a while; the service is not
        # advertised until the node is up.
        self.declare_parameter("service_timeout", 60.0)
        self.declare_parameter("max_attempts", 5)
        self.declare_parameter("retry_delay", 2.0)

        self.ok = False

    def get(self, name):
        return self.get_parameter(name).value

    def already_running(self):
        """True if a non-frozen trajectory already exists.

        Guards against creating a second active trajectory if this is run twice
        (e.g. launch retry plus a manual call). Two active trajectories in pure
        localization produce two competing map->odom solutions.
        """
        cli = self.create_client(GetTrajectoryStates, "/get_trajectory_states")
        if not cli.wait_for_service(timeout_sec=5.0):
            return False
        fut = cli.call_async(GetTrajectoryStates.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.result() is None:
            return False

        states = fut.result().trajectory_states
        # TrajectoryState: ACTIVE=0, FINISHED=1, FROZEN=2, DELETED=3
        active = [tid for tid, st in
                  zip(states.trajectory_id, states.trajectory_state)
                  if st == 0]
        if active:
            self.get_logger().warn(
                "trajectory {} is already ACTIVE; not starting another"
                .format(active))
            return True
        return False

    def build_request(self):
        req = StartTrajectory.Request()
        req.configuration_directory = self.get("configuration_directory")
        req.configuration_basename = self.get("configuration_basename")
        req.use_initial_pose = True
        req.relative_to_trajectory_id = int(
            self.get("relative_to_trajectory_id"))

        yaw = float(self.get("start_yaw"))
        req.initial_pose.position.x = float(self.get("start_x"))
        req.initial_pose.position.y = float(self.get("start_y"))
        req.initial_pose.position.z = 0.0
        # Planar, so only z/w are non-trivial.
        req.initial_pose.orientation.x = 0.0
        req.initial_pose.orientation.y = 0.0
        req.initial_pose.orientation.z = math.sin(0.5 * yaw)
        req.initial_pose.orientation.w = math.cos(0.5 * yaw)
        return req

    def run(self):
        if not self.get("configuration_directory"):
            self.get_logger().error("configuration_directory is empty")
            return False

        cli = self.create_client(StartTrajectory, "/start_trajectory")
        timeout = float(self.get("service_timeout"))
        self.get_logger().info(
            "waiting up to {:.0f}s for /start_trajectory ...".format(timeout))
        if not cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                "/start_trajectory never appeared. Is cartographer_node "
                "running? Check for a FATAL in its output.")
            return False

        if self.already_running():
            return True

        req = self.build_request()
        self.get_logger().info(
            "starting trajectory at x={:.3f} y={:.3f} yaw={:.3f} rad "
            "(config {})".format(req.initial_pose.position.x,
                                 req.initial_pose.position.y,
                                 float(self.get("start_yaw")),
                                 req.configuration_basename))

        attempts = int(self.get("max_attempts"))
        delay = float(self.get("retry_delay"))
        for i in range(1, attempts + 1):
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
            res = fut.result()

            if res is None:
                self.get_logger().warn(
                    "attempt {}/{}: no response".format(i, attempts))
            elif res.status.code == 0:
                self.get_logger().info(
                    "trajectory {} started. {}".format(
                        res.trajectory_id, res.status.message))
                return True
            else:
                self.get_logger().error(
                    "attempt {}/{}: code={} {}".format(
                        i, attempts, res.status.code, res.status.message))

            if i < attempts:
                self.get_clock().sleep_for(
                    rclpy.duration.Duration(seconds=delay))

        self.get_logger().error(
            "giving up. Without a trajectory there is no map frame, and every "
            "downstream node will report state_age=inf.")
        return False


def main():
    rclpy.init()
    node = TrajectoryStarter()
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
