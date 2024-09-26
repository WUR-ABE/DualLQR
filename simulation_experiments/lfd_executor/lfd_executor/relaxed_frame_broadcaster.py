from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


class FixedFrameBroadcaster(Node):
    def __init__(self):
        super().__init__("relaxed_frame_broadcaster")

        # Declare and acquire child_frame_name parameter
        self.declare_parameter("tf_learned_base_frame_topic", "relaxed_ik")
        self.child_frame_name = (
            self.get_parameter("tf_learned_base_frame_topic")
            .get_parameter_value()
            .string_value
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.broadcast_timer_callback)

    def broadcast_timer_callback(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = self.child_frame_name
        t.transform.translation.x = 0.315444
        t.transform.translation.y = -0.164957
        t.transform.translation.z = 0.640821
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FixedFrameBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
