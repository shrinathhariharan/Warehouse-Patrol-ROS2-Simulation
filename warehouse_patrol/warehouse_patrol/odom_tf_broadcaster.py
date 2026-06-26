"""Broadcast odometry poses as TF for RViz."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    """Publishes the odom -> base_link transform from the bridged odometry topic."""

    def __init__(self) -> None:
        super().__init__("odom_tf_broadcaster")
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value
        self.parent_frame = self.declare_parameter("parent_frame", "odom").value
        self.child_frame = self.declare_parameter("child_frame", "base_link").value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 10)
        self.get_logger().info(
            f"Broadcasting TF from {self.odom_topic}: "
            f"{self.parent_frame} -> {self.child_frame}"
        )

    def _odom_callback(self, msg: Odometry) -> None:
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = msg.header.frame_id or self.parent_frame
        transform.child_frame_id = msg.child_frame_id or self.child_frame
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
