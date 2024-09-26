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
import math as m
import transforms3d as t3d
import numpy as np

# Import message types
from lfd_executor_msgs.action import ControlledGoalPose
from geometry_msgs.msg import TransformStamped

# TF subscriber imports
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster


class ControlledGoalPoseActionServer(Node):
    def __init__(self):
        # Create rospy node
        super().__init__("controlled_goal_pose_broadcaster")

        # Set default callback to allow for multithreading TF
        self._default_callback_group = ReentrantCallbackGroup()

        # Initialize transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize action server
        self.action_server = ActionServer(
            self,
            ControlledGoalPose,
            "action_controlled_goal_pose",
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
        )

    def broadcast_timer_callback(self):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        current_time = self.get_clock().now()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.header.frame_id
        t.child_frame_id = self.child_frame_id

        # Determine disturbance transformation
        disturbance_matrix = np.identity(4)
        if self.noise_type == "step":
            noise = self.noise_level * ((self.noise_rate * 1e9 * 49.0 / 3.0 + self.start_time.nanoseconds) < current_time.nanoseconds)
            if self.noise_dimension == "x":
                disturbance_matrix[0, 3] += noise
            elif self.noise_dimension == "yaxis":
                disturbance_matrix[1, 3] += noise
            elif self.noise_dimension == "z":
                disturbance_matrix[2, 3] += noise
            else:
                roll = 0.0
                pitch = 0.0
                yaw = 0.0
                if self.noise_dimension == "roll":
                    roll = noise
                elif self.noise_dimension == "pitch":
                    pitch = noise
                elif self.noise_dimension == "yaw":
                    yaw = noise
                else:
                    self.get_logger().error("Noise dimension not recognized")
                disturbance_matrix[0:3, 0:3] = t3d.euler.euler2mat(roll, pitch, yaw, axes="sxyz")
        elif self.noise_type == "sinus":
            noise = self.noise_level * m.sin(self.noise_rate * current_time.nanoseconds / 1e9)
            if self.noise_dimension == "x":
                disturbance_matrix[0, 3] += noise
            elif self.noise_dimension == "yaxis":
                disturbance_matrix[1, 3] += noise
            elif self.noise_dimension == "z":
                disturbance_matrix[2, 3] += noise
            else:
                roll = 0.0
                pitch = 0.0
                yaw = 0.0
                if self.noise_dimension == "roll":
                    roll = noise
                elif self.noise_dimension == "pitch":
                    pitch = noise
                elif self.noise_dimension == "yaw":
                    yaw = noise
                else:
                    self.get_logger().error("Noise dimension not recognized")
                disturbance_matrix[0:3, 0:3] = t3d.euler.euler2mat(roll, pitch, yaw, axes="sxyz")
        elif self.noise_type == "none":
            pass
        else:
            self.get_logger().error("Unknown noise type: {0}".format(self.noise_type))
            self.get_logger().error("No noise will be added to the transformation")

        # Multiply the default transformation with the disturbance transformation
        combined_matrix = np.dot(self.default_matrix, disturbance_matrix)

        # Convert the matrix to a transformation message
        t.transform.translation.x = combined_matrix[0, 3]
        t.transform.translation.y = combined_matrix[1, 3]
        t.transform.translation.z = combined_matrix[2, 3]
        # Matrix to quaternion conversion
        quaternion = t3d.quaternions.mat2quat(combined_matrix[0:3, 0:3])
        t.transform.rotation.x = quaternion[1]
        t.transform.rotation.y = quaternion[2]
        t.transform.rotation.z = quaternion[3]
        t.transform.rotation.w = quaternion[0]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

    def cancel_callback(self, goal_handle):
        # Accepts or rejects a client request to cancel an action
        self.get_logger().info("Canceling controlled goal pose")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        # Get goal
        goal = goal_handle.request

        # Store header of transformation
        self.header = goal.transformation.header
        self.child_frame_id = goal.transformation.child_frame_id

        # Determine noise
        self.noise_type = goal.noise_type
        self.noise_level = goal.noise_level
        self.noise_rate = goal.noise_rate
        self.noise_dimension = goal.noise_dimension

        self.start_time = self.get_clock().now()

        # Set the default transformation matrix of the object
        translation = goal.transformation.transform.translation
        rotation = goal.transformation.transform.rotation
        rotation_matrix = t3d.quaternions.quat2mat((rotation.w, rotation.x, rotation.y, rotation.z))
        default_matrix = np.eye(4)
        default_matrix[0:3, 0:3] = rotation_matrix
        default_matrix[0:3, 3] = np.asarray((translation.x, translation.y, translation.z))

        # Make it global
        self.default_matrix = default_matrix

        # Start timer to publish the transformation
        self.timer = self.create_timer(0.005, self.broadcast_timer_callback)

        # Wait for the action to be cancelled
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal cancelled")
                goal_handle.canceled()
                self.timer.cancel()
                break
            time.sleep(0.1)

        result = ControlledGoalPose.Result()
        result.shut_down = True
        return result


def main():
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()

    handle_dynamic_frame_broadcaster = ControlledGoalPoseActionServer()

    executor.add_node(handle_dynamic_frame_broadcaster)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()
