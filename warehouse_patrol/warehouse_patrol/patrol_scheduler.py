"""High-level warehouse patrol mission scheduler."""

from __future__ import annotations

import math
from enum import Enum, auto
from typing import Dict, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class PatrolState(Enum):
    IDLE = auto()
    NAVIGATING = auto()
    AT_WAYPOINT = auto()
    PATROL_LOOP_COMPLETE = auto()


class PatrolScheduler(Node):
    """Sequentially visits warehouse room waypoints and tracks patrol telemetry."""

    ROOM_WAYPOINTS: Dict[str, Tuple[float, float]] = {
        "loading_dock": (1.5, 1.5),
        "aisle_a": (4.0, 1.5),
        "storage_a": (7.0, 1.5),
        "aisle_b": (4.0, 4.5),
        "storage_b": (7.0, 4.5),
        "office": (1.5, 4.5),
    }

    def __init__(self) -> None:
        super().__init__("patrol_scheduler")

        self.waypoint_tolerance = self.declare_parameter("waypoint_tolerance", 0.35).value
        self.max_linear_speed = self.declare_parameter("max_linear_speed", 0.35).value
        self.max_angular_speed = self.declare_parameter("max_angular_speed", 0.9).value
        self.waypoint_dwell_sec = self.declare_parameter("waypoint_dwell_sec", 2.0).value
        self.patrol_loops = self.declare_parameter("patrol_loops", 0).value  # 0 = infinite

        self.state = PatrolState.IDLE
        self.waypoint_names = list(self.ROOM_WAYPOINTS.keys())
        self.current_index = 0
        self.completed_loops = 0
        self.waypoints_visited = 0
        self.distance_traveled = 0.0
        self.dwell_remaining = 0.0

        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.have_odom = False
        self.last_odom_x = 0.0
        self.last_odom_y = 0.0

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_input", 10)
        self.telemetry_pub = self.create_publisher(String, "/patrol/telemetry", 10)

        odom_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, "/odom", self._odom_callback, odom_qos)
        self.create_timer(0.1, self._control_loop)
        self.create_timer(5.0, self._publish_telemetry)

        self.get_logger().info(
            f"Patrol scheduler ready with {len(self.waypoint_names)} room waypoints"
        )
        self._begin_patrol()

    def _begin_patrol(self) -> None:
        self.state = PatrolState.NAVIGATING
        self.current_index = 0
        target = self.waypoint_names[self.current_index]
        self.get_logger().info(f"Starting patrol. First target: {target}")

    def _odom_callback(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.have_odom:
            dx = x - self.last_odom_x
            dy = y - self.last_odom_y
            self.distance_traveled += math.hypot(dx, dy)

        self.last_odom_x = x
        self.last_odom_y = y
        self.pose_x = x
        self.pose_y = y
        self.have_odom = True

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.pose_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _current_target(self) -> Tuple[str, Tuple[float, float]]:
        name = self.waypoint_names[self.current_index]
        return name, self.ROOM_WAYPOINTS[name]

    def _advance_waypoint(self) -> None:
        self.waypoints_visited += 1
        self.current_index += 1

        if self.current_index >= len(self.waypoint_names):
            self.completed_loops += 1
            self.current_index = 0
            self.state = PatrolState.PATROL_LOOP_COMPLETE
            self.get_logger().info(
                f"Completed patrol loop #{self.completed_loops} "
                f"({self.waypoints_visited} waypoints, "
                f"{self.distance_traveled:.1f} m traveled)"
            )

            if self.patrol_loops > 0 and self.completed_loops >= self.patrol_loops:
                self.state = PatrolState.IDLE
                self.get_logger().info("Patrol mission complete.")
                return

            self.state = PatrolState.NAVIGATING
            self.get_logger().info("Starting next patrol loop.")

        target_name, _ = self._current_target()
        self.get_logger().info(f"Next room: {target_name}")

    def _control_loop(self) -> None:
        cmd = Twist()

        if not self.have_odom or self.state == PatrolState.IDLE:
            self.cmd_pub.publish(cmd)
            return

        if self.state == PatrolState.AT_WAYPOINT:
            self.dwell_remaining -= 0.1
            if self.dwell_remaining <= 0.0:
                self._advance_waypoint()
            self.cmd_pub.publish(cmd)
            return

        if self.state in (PatrolState.NAVIGATING, PatrolState.PATROL_LOOP_COMPLETE):
            self.state = PatrolState.NAVIGATING
            target_name, (target_x, target_y) = self._current_target()

            dx = target_x - self.pose_x
            dy = target_y - self.pose_y
            distance = math.hypot(dx, dy)

            if distance < self.waypoint_tolerance:
                self.state = PatrolState.AT_WAYPOINT
                self.dwell_remaining = self.waypoint_dwell_sec
                self.get_logger().info(f"Arrived at {target_name}. Dwelling briefly.")
                self.cmd_pub.publish(cmd)
                return

            desired_heading = math.atan2(dy, dx)
            heading_error = self._normalize_angle(desired_heading - self.pose_yaw)

            cmd.angular.z = max(
                -self.max_angular_speed,
                min(self.max_angular_speed, 2.0 * heading_error),
            )

            if abs(heading_error) < 0.35:
                cmd.linear.x = min(self.max_linear_speed, 0.8 * distance)
            else:
                cmd.linear.x = 0.05

            self.cmd_pub.publish(cmd)

    def _publish_telemetry(self) -> None:
        target_name, _ = self._current_target()
        msg = String()
        msg.data = (
            f"state={self.state.name} "
            f"target={target_name} "
            f"loops={self.completed_loops} "
            f"visited={self.waypoints_visited} "
            f"distance_m={self.distance_traveled:.2f} "
            f"pose=({self.pose_x:.2f},{self.pose_y:.2f})"
        )
        self.telemetry_pub.publish(msg)
        self.get_logger().info(f"Telemetry: {msg.data}")

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolScheduler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
