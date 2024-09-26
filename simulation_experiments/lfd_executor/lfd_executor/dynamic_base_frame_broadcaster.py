#!/usr/bin/env python3

# This file listens for the command to start broadcasting the dynamic base frame
# It will publish the transformation at a fixed rate, linked to the object frame

# Author: Robert van de Ven
# Email: robert.vandeven@wur.nl

# Import modules
import rclpy
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
import time

# Import message types
from lfd_executor_msgs.action import DynamicBaseFrame
from geometry_msgs.msg import TransformStamped

# TF subscriber imports
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster


class DynamicBaseFrameActionServer(Node):
    def __init__(self):
        # Create rospy node
        super().__init__("dynamic_base_frame_broadcaster")

        # Set default callback to allow for multithreading TF
        self._default_callback_group = ReentrantCallbackGroup()

        # Initialize transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize action server
        self.action_server = ActionServer(
            self,
            DynamicBaseFrame,
            "action_dynamic_base_frame",
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
        )

    def broadcast_timer_callback(self):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.transformation.header.frame_id
        t.child_frame_id = self.transformation.child_frame_id

        # Set the default position of the object
        t.transform.translation.x = self.transformation.transform.translation.x
        t.transform.translation.y = self.transformation.transform.translation.y
        t.transform.translation.z = self.transformation.transform.translation.z
        t.transform.rotation.x = self.transformation.transform.rotation.x
        t.transform.rotation.y = self.transformation.transform.rotation.y
        t.transform.rotation.z = self.transformation.transform.rotation.z
        t.transform.rotation.w = self.transformation.transform.rotation.w

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

    def cancel_callback(self, goal_handle):
        # Accepts or rejects a client request to cancel an action
        # self.get_logger().info("Canceling dynamic base frame")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        # Get goal
        goal = goal_handle.request

        # Get transformation
        self.transformation = goal.transformation

        # Start time to publish the transformation
        self.timer = self.create_timer(0.01, self.broadcast_timer_callback)

        # Wait for the action to be cancelled
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal cancelled")
                goal_handle.canceled()
                self.timer.cancel()
                break
            time.sleep(0.1)

        result = DynamicBaseFrame.Result()
        result.shut_down = True
        return result


def main():
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()

    handle_dynamic_frame_broadcaster = DynamicBaseFrameActionServer()

    executor.add_node(handle_dynamic_frame_broadcaster)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
