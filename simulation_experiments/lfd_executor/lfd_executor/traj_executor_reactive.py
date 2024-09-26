#!/usr/bin/env python3

# This file listens for the command to start executing a trajectory
# It will publish goal poses at a define rate

# Inputs: JointStates from UR, target object pose

# Author: Robert van de Ven
# Email: robert.vandeven@wur.nl

# Import modules
import rclpy
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
import numpy as np
import pickle
import transforms3d as t3d
import pbdlib as pbd
import sys
import time
import roboticstoolbox as rtb
from scipy.linalg import solve_discrete_are as solve_algebraic_riccati_discrete

# Locally import UR5e model
from .rtb_model.UR5e import UR5e

# Import message types
from lfd_executor_msgs.action import TrajectoryExecuteReactive, DynamicBaseFrame
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped

# TF subscriber imports
from tf2_ros.buffer import Buffer
from tf2_ros.buffer_interface import BufferInterface
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException

# Rel import
from .pbdlib_custom import gmmr

# Fix the change in structure between pickle and ROS2 structure
sys.modules["gmmr"] = gmmr


class TrajectoryActionServer(Node):
    def __init__(self):
        # Create rospy node
        super().__init__("trajectory_executor_reactive")

        # Set default callback to allow for multithreading TF
        self._default_callback_group = ReentrantCallbackGroup()

        # Get params
        self.declare_parameter(
            "publish_rate",
            125.0,
            ParameterDescriptor(description="Frequency of running controller"),
        )
        self.declare_parameter(
            "controller_rate",
            6.0,
            ParameterDescriptor(description="Frequency of stored controller"),
        )
        self.declare_parameter(
            "tf_object_topic",
            "cube_ik",
            ParameterDescriptor(description="Topic of object pose"),
        )
        self.declare_parameter(
            "tf_manipulator_topic",
            "tool_ik",
            ParameterDescriptor(description="Topic of manipulator pose"),
        )
        self.declare_parameter(
            "tf_learned_base_frame_topic",
            "relaxed_ik",
            ParameterDescriptor(description="Topic of learned base frame"),
        )
        self.declare_parameter(
            "tf_robot_base_topic",
            "base_link",
            ParameterDescriptor(description="Topic of robot base frame"),
        )
        self.declare_parameter(
            "controller_type",
            "DualLQR",
            ParameterDescriptor(description="Use multiple frames in controller"),
        )
        self.declare_parameter(
            "data_folder",
            "/media/data/OneDrive/PhD/03 Study 2/03 Software/trajectories/",
            ParameterDescriptor(description="Folder to store data in"),
        )

        # Helper variables
        self.publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        self.controller_rate = self.get_parameter("controller_rate").get_parameter_value().double_value
        self.object_topic = self.get_parameter("tf_object_topic").get_parameter_value().string_value
        self.manipulator_topic = self.get_parameter("tf_manipulator_topic").get_parameter_value().string_value
        self.relaxed_frame = self.get_parameter("tf_learned_base_frame_topic").get_parameter_value().string_value
        self.robot_base = self.get_parameter("tf_robot_base_topic").get_parameter_value().string_value
        self.controller_type = self.get_parameter("controller_type").get_parameter_value().string_value
        self.data_folder = self.get_parameter("data_folder").get_parameter_value().string_value

        # Load UR5e model for IK solver
        self.ur5e = UR5e()
        self.ur5e_ets = self.ur5e.ets()

        # Create publishers
        self.joint_pos_pub = self.create_publisher(Float64MultiArray, "/forward_position_controller/commands", 1)

        # Create TF subscribers
        self.tf_interface = BufferInterface()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Create subscriber for joint states
        self.joint_positions = None
        self.joint_sub = self.create_subscription(JointState, "/joint_states", self.joint_callback, 10)
        self.joint_sub

        # Create action client
        self.action_client = ActionClient(self, DynamicBaseFrame, "action_dynamic_base_frame")

        # Create action server
        self.server = ActionServer(
            self,
            TrajectoryExecuteReactive,
            "action_traj_exec_react",
            self.send_command,
            cancel_callback=self.cancel_callback,
        )

        # self.get_logger().info("Trajectory executor reactive action server started")

    def joint_callback(self, msg):
        # Callback for joint positions
        unsorted_joint_positions = msg.position
        self.joint_positions = np.asarray(
            [
                unsorted_joint_positions[5],
                unsorted_joint_positions[0],
                unsorted_joint_positions[1],
                unsorted_joint_positions[2],
                unsorted_joint_positions[3],
                unsorted_joint_positions[4],
            ]
        )

    def cancel_callback(self, goal_handle):
        # Accepts or rejects a client request to cancel an action
        # self.get_logger().info("Canceling trajectory execution")
        return CancelResponse.ACCEPT

    def tf_pose_getter(self, frame, child_frame):
        # Get transform between base and tcp
        try:
            pose_transformation = self.tf_buffer.lookup_transform(frame, child_frame, time=rclpy.time.Time())
            return pose_transformation
        except TransformException as ex:
            # Set identity transformation
            pose_transformation = TransformStamped()
            pose_transformation.transform.translation.x = 0.0
            pose_transformation.transform.translation.y = 0.0
            pose_transformation.transform.translation.z = 0.0
            pose_transformation.transform.rotation.x = 0.0
            pose_transformation.transform.rotation.y = 0.0
            pose_transformation.transform.rotation.z = 0.0
            pose_transformation.transform.rotation.w = 1.0
            pose_transformation.header.frame_id = frame
            pose_transformation.child_frame_id = child_frame
            self.get_logger().warn("Transform not found between {0} and {1}, using identity transform".format(frame, child_frame))
            return pose_transformation

    def tf_pose_array_getter(self, frame, child_frame):
        transformation = self.tf_pose_getter(frame, child_frame)
        x, y, z = (
            transformation.transform.translation.x,
            transformation.transform.translation.y,
            transformation.transform.translation.z,
        )
        roll, pitch, yaw = t3d.euler.quat2euler(
            (
                transformation.transform.rotation.w,
                transformation.transform.rotation.x,
                transformation.transform.rotation.y,
                transformation.transform.rotation.z,
            ),
            axes="sxyz",
        )
        pose = np.asarray((x, y, z, roll, pitch, yaw))
        return pose

    def tf_to_matrix(self, transform_stamped):
        # Convert transform to matrix
        translation = transform_stamped.transform.translation
        rotation = transform_stamped.transform.rotation
        rotation_matrix = t3d.quaternions.quat2mat((rotation.w, rotation.x, rotation.y, rotation.z))
        matrix = np.eye(4)
        matrix[0:3, 0:3] = rotation_matrix
        matrix[0:3, 3] = np.asarray((translation.x, translation.y, translation.z))

        return matrix

    def array_to_matrix(self, array, transformation, tool_ik_transform=False):
        # Get matrix from transformation
        trans_matrix_frame_child = self.tf_to_matrix(transformation)

        # Get tranformation matrix from array
        rot_array = t3d.euler.euler2mat(array[3], array[4], array[5], axes="sxyz")
        trans_matrix_array = np.eye(4)
        trans_matrix_array[0:3, 0:3] = rot_array
        trans_matrix_array[0:3, 3] = array[0:3]

        # Get tranformation matrix from frame to array
        if tool_ik_transform:
            tool_transformation = np.eye(4)
            tool_rot = t3d.quaternions.quat2mat((0.0, 0.0, 0.707, -0.707))
            tool_transformation[0:3, 0:3] = tool_rot

            combined_transform = np.dot(
                np.dot(trans_matrix_frame_child, trans_matrix_array),
                np.linalg.inv(tool_transformation),
            )
        else:
            combined_transform = np.dot(trans_matrix_frame_child, trans_matrix_array)

        return combined_transform

    def transform_control(self, control, frame, child_frame):
        # Get rotation matrix between frame and child_frame
        transformation = self.tf_pose_getter(frame, child_frame)
        matrix = t3d.quaternions.quat2mat(
            (
                transformation.transform.rotation.w,
                transformation.transform.rotation.x,
                transformation.transform.rotation.y,
                transformation.transform.rotation.z,
            )
        )

        # Transform position vector to robot base frame
        pos_vec = control[0:3].reshape((3, 1))
        pos_tvec = np.dot(matrix, pos_vec)

        # Transform orientation vector to robot base frame
        rot_vec = control[3:6].reshape((3, 1))
        rot_tvec = np.dot(matrix, rot_vec)

        # Combine position and orientation
        transformed_control = np.vstack((pos_tvec, rot_tvec)).flatten()
        return transformed_control

    def timer_callback(self):
        # Time since started
        current_time = self.get_clock().now()
        diff = current_time - self.start_time

        # Fraction of time
        time_fract = diff.nanoseconds / self.duration.nanoseconds

        # Integer time step
        t = int(time_fract * (self.lqr_end.horizon - 2))

        if t > self.lqr_end.horizon - 2:
            t = self.lqr_end.horizon - 2

        # Get current state
        if self.controller_type == "InfLQR":
            pose_i = self.tf_pose_array_getter(self.relaxed_frame, self.manipulator_topic)
            base_transform_i = self.tf_pose_getter(self.robot_base, self.moving_frame)
            self.xis += [pose_i]  # Internal feedback: [self.xi_g[-1][:-1]]
        else:
            pose_i = self.tf_pose_array_getter(self.moving_frame, self.manipulator_topic)
            base_transform_i = self.tf_pose_getter(self.robot_base, self.moving_frame)
            self.xis += [pose_i]  # Internal feedback: [self.xi_g[-1][:-1]]

        if self.controller_type == "DualLQR":
            # Get current state in start frame
            pose_i_start = self.tf_pose_array_getter(self.relaxed_frame, self.manipulator_topic)

            # Determine both control inputs
            self.us_start += [-self.lqr_start._K[t].dot(pose_i_start) + self.lqr_start._Kv[t].dot(self.lqr_start._v[t + 1])]
            self.us_end += [-self.lqr_end._K[t].dot(self.xis[-1]) + self.lqr_end._Kv[t].dot(self.lqr_end._v[t + 1])]

            # Get rotation matrix between robot base frame and relaxed_ik frame
            transformed_us_start = self.transform_control(self.us_start[-1], self.moving_frame, self.relaxed_frame)

            # Get both inverse covariance matrices
            lambda_start, _ = self.lqr_start.get_Q_z(t)
            lambda_end, _ = self.lqr_end.get_Q_z(t)

            # Make everything but diagonal zero
            lambda_start = np.diag(np.diag(lambda_start))
            lambda_end = np.diag(np.diag(lambda_end))

            # Compute product of both control inputs
            weighted_us = np.linalg.inv(lambda_start + lambda_end).dot(lambda_start.dot(transformed_us_start) + lambda_end.dot(self.us_end[-1]))
            self.us += [weighted_us]
        elif self.controller_type == "InfLQR":
            # Get current state of goal pose
            pose_n = self.tf_pose_array_getter(self.relaxed_frame, self.object_topic)

            # Create arrays for transforming data
            A0 = np.identity(n=7)
            An = np.identity(n=7)
            b0 = np.zeros(7)
            bn = np.zeros(7)
            A0[1:7, 1:7], b0[1:7] = pbd.utils.inv_for_lintrans(self.initial_pose)
            An[1:7, 1:7], bn[1:7] = pbd.utils.inv_for_lintrans(pose_n)

            # Split models
            _mod1 = self.model.gmm_.marginal_array(self.dim1).lintrans(A0, b0)
            _mod2 = self.model.gmm_.marginal_array(self.dim2).lintrans(An, bn)

            # Combine models
            _prod = _mod1 * _mod2

            # Get the most probable trajectory for this initial and final pose
            mu, sigma = _prod.condition(self.time_axis[:, None], dim_in=slice(0, 1), dim_out=slice(0, 7))

            # Update LQR
            self.lqr_end.gmm_xi = [mu[:, 1:], sigma[:, 1:, 1:], self.sq]

            # Store mu & sigma of this timestep
            self.store_model += [[self.lqr_end.gmm_xi[0][t], self.lqr_end.gmm_xi[1][t]]]

            # Infinite horizon self.lqr_end stuff
            R = self.lqr_end.get_R(t)  # Control cost matrix

            # Covariance of this state
            Q = np.linalg.inv(self.lqr_end.gmm_xi[1][t])

            P = solve_algebraic_riccati_discrete(self.lqr_end.A, self.lqr_end.B, Q, R)

            L = np.linalg.inv(self.lqr_end.B.T @ P @ self.lqr_end.B + R) @ self.lqr_end.B.T @ P @ self.lqr_end.A  # Feedback gain (discrete version)

            # Control input based on current pose
            self.us_start = [L @ (self.lqr_end.gmm_xi[0][t] - self.xis[-1])]

            # Get rotation matrix between robot base frame and relaxed_ik frame
            transformed_us_start = self.transform_control(self.us_start[-1], self.moving_frame, self.relaxed_frame)
            self.us += [transformed_us_start]
        else:
            raise ValueError("Controller type not supported")

        # Goal pose, based on current pose and control input based on current pose
        # Add time fraction to goal pose
        if self.controller_type == "InfLQR":
            self.xi_g += [
                np.append(
                    self.lqr_end.A.dot(self.xis[-1]) + self.lqr_end.B.dot(self.us_start[-1]),
                    time_fract,
                )
            ]  # self.xi_g[-1][:-1] should be self.xis[-1]

            # Determine goal pose in robot base frame
            goal_matrix = self.array_to_matrix(
                self.xi_g[-1][:-1],
                self.tf_pose_getter(self.robot_base, self.relaxed_frame),
                tool_ik_transform=True,
            )
        else:
            self.xi_g += [
                np.append(
                    self.lqr_end.A.dot(self.xis[-1]) + self.lqr_end.B.dot(self.us[-1]),
                    time_fract,
                )
            ]  # self.xi_g[-1][:-1] should be self.xis[-1]

            # Determine goal pose in robot base frame
            goal_matrix = self.array_to_matrix(
                self.xi_g[-1][:-1],
                base_transform_i,
                tool_ik_transform=True,
            )

        # Use TracIK to solve IK
        joint_positions, search_success, iterations, searches, residual = self.ur5e_ets.ik_LM(
            goal_matrix,
            q0=self.joint_positions,
            ilimit=30,  # Maximum iterations allowed in a search for a solution
            slimit=100,  # Maximum searches allowed for a solution
            tol=1e-6,  # Solution tolerance
            k=0.1,
            joint_limits=False,
            method="chan",
        )

        try:
            # Fill the Float64MultiArray with the joint positions
            joint_goals = Float64MultiArray()
            joint_goals.data = list(joint_positions)

            # Publish joint goals
            self.joint_pos_pub.publish(joint_goals)
        except:
            self.get_logger().info("No joint goals published")
            joint_positions = [50.0, 50.0, 50.0, 50.0, 50.0, 50.0]

        # Store controls
        self.controls += [
            np.hstack((current_time.nanoseconds, self.xis[-1], self.us[-1], self.xi_g[-1], joint_positions, search_success, iterations, searches, residual))
        ]

    def send_command(self, goal_handle):
        # Set success boolean
        success = True

        ## Set up dynamic frame publisher to publish the dynamic relaxed frame
        # Creat goal message
        goal_msg = DynamicBaseFrame.Goal()
        goal_transformation = self.tf_pose_getter(self.object_topic, self.relaxed_frame)
        self.moving_frame = "dynamic_base_frame"
        goal_transformation.child_frame_id = self.moving_frame
        goal_msg.transformation = goal_transformation

        # Make sure action server is available
        self.action_client.wait_for_server()

        # Send goal
        self.dynamic_goal_future = self.action_client.send_goal_async(goal_msg)

        # Make sure goal is being published
        # self.get_logger().info("Waiting for dynamic base frame to be published")
        transform_waiter = self.tf_buffer.wait_for_transform_async(self.moving_frame, self.object_topic, time=rclpy.time.Time())
        while not transform_waiter.done():
            time.sleep(0.2)
        # self.get_logger().info("Dynamic base frame published")

        # Get transform between base and tcp
        pose_0 = self.tf_pose_array_getter(self.moving_frame, self.manipulator_topic)
        # self.get_logger().info("Initial pose: {0}".format(pose_0))

        self.initial_pose = pose_0

        # Convert to matrix
        base_transform = self.tf_pose_getter(self.robot_base, self.moving_frame)
        matrix_0 = self.array_to_matrix(pose_0, base_transform, tool_ik_transform=True)

        # Calculate IK
        joint_positions, search_success, iterations, searches, residual = self.ur5e_ets.ik_LM(
            matrix_0,
            q0=self.joint_positions,
            ilimit=30,  # Maximum iterations allowed in a search for a solution
            slimit=100,  # Maximum searches allowed for a solution
            tol=1e-6,  # Solution tolerance
            k=0.1,
            joint_limits=False,
            method="chan",
        )

        # Get transform between base and goal
        pose_n = self.tf_pose_array_getter(self.moving_frame, self.object_topic)
        # self.get_logger().info("Final pose: {0}".format(pose_n))

        # Load model from file location
        self.model = pickle.load(open(goal_handle.request.model_name, "rb"), encoding="latin1")

        # Predict trajectory (outside of model to also get the sigma)
        # Create arrays for transforming data
        A0 = np.identity(n=7)
        An = np.identity(n=7)
        b0 = np.zeros(7)
        bn = np.zeros(7)
        A0[1:7, 1:7], b0[1:7] = pbd.utils.inv_for_lintrans(pose_0)
        An[1:7, 1:7], bn[1:7] = pbd.utils.inv_for_lintrans(pose_n)

        # Get time at 125 Hz
        length = int(self.publish_rate * (len(self.model.t) / self.controller_rate))
        self.time_axis = np.linspace(0, 100, length)

        # Set required variables for LQR
        A, b = pbd.utils.get_canonical(6, 1, 1.0 / self.publish_rate)
        self.sq = [i for i in range(0, len(self.time_axis))]

        # Select columns for split models
        self.dim1 = np.array([0, 1, 2, 3, 4, 5, 6])
        self.dim2 = np.array([0, 7, 8, 9, 10, 11, 12])

        # Put in initial pose
        self.xis = [pose_0]

        # Initialize first control input
        self.us = [np.zeros(6)]

        # List of goal states, initial goal is start pose
        # Add time to start pose
        init_goal = np.append(pose_0, 0)
        self.xi_g = [init_goal]

        # Set up LQR based on controller_type
        if self.controller_type == "DualLQR":
            # Split models
            _mod1 = self.model.gmm_.marginal_array(self.dim1).lintrans(A0, b0)
            _mod2 = self.model.gmm_.marginal_array(self.dim2).lintrans(An, bn)

            # Get the most probable trajectory and uncertainty
            mu1, sigma1 = _mod1.condition(self.time_axis[:, None], dim_in=slice(0, 1), dim_out=slice(0, 7))
            mu2, sigma2 = _mod2.condition(self.time_axis[:, None], dim_in=slice(0, 1), dim_out=slice(0, 7))

            # Set up LQR
            self.lqr_start = pbd.LQR(A, b, dt=1.0 / self.publish_rate, horizon=len(mu1))
            self.lqr_start.gmm_xi = [mu1[:, 1:], sigma1[:, 1:, 1:], self.sq]
            self.lqr_start.gmm_u = goal_handle.request.reg_factor

            self.lqr_end = pbd.LQR(A, b, 1.0 / self.publish_rate, horizon=len(mu2))
            self.lqr_end.gmm_xi = [mu2[:, 1:], sigma2[:, 1:, 1:], self.sq]
            self.lqr_end.gmm_u = goal_handle.request.reg_factor

            # Fit LQR
            # self.get_logger().info("Fitting LQR")
            self.lqr_start.ricatti()
            self.lqr_end.ricatti()

            # Initialize first control input
            self.us_start = [-self.lqr_start._K[0].dot(pose_0) + self.lqr_start._Kv[0].dot(self.lqr_start._v[0])]
            self.us_end = [-self.lqr_end._K[0].dot(pose_0) + self.lqr_end._Kv[0].dot(self.lqr_end._v[0])]
        elif self.controller_type == "InfLQR":
            # Split models
            _mod1 = self.model.gmm_.marginal_array(self.dim1).lintrans(A0, b0)
            _mod2 = self.model.gmm_.marginal_array(self.dim2).lintrans(An, bn)

            # Combine models
            _prod = _mod1 * _mod2

            # Get the most probable trajectory for this initial and final pose
            mu, sigma = _prod.condition(self.time_axis[:, None], dim_in=slice(0, 1), dim_out=slice(0, 7))
            self.get_logger().info("Mu shape: {0}".format(mu.shape))

            # Set up LQR
            self.lqr_end = pbd.LQR(A, b, 1.0 / self.publish_rate, horizon=len(mu))
            self.lqr_end.gmm_xi = [mu[:, 1:], sigma[:, 1:, 1:], self.sq]
            self.lqr_end.gmm_u = goal_handle.request.reg_factor

            # Initialize first time
            t = 0

            # Infinite horizon self.lqr_end stuff
            R = self.lqr_end.get_R(t)  # Control cost matrix

            # Covariance of this state
            Q = self.lqr_end.gmm_xi[1][t]

            P = solve_algebraic_riccati_discrete(self.lqr_end.A, self.lqr_end.B, Q, R)

            L = np.linalg.inv(self.lqr_end.B.T @ P @ self.lqr_end.B + R) @ self.lqr_end.B.T @ P @ self.lqr_end.A  # Feedback gain (discrete version)

            # Control input based on current pose
            self.us_start = [L @ (self.lqr_end.gmm_xi[0][t] - self.xis[-1])]

            # Store mu & sigma of this timestep
            self.store_model = [self.lqr_end.gmm_xi[0][t], self.lqr_end.gmm_xi[1][t]]
        else:
            self.get_logger().error("Invalid controller type")
            return

        # Set feedback and result message
        self.feedback = TrajectoryExecuteReactive.Feedback()
        result = TrajectoryExecuteReactive.Result()

        # Get current time
        self.start_time = self.get_clock().now()
        self.duration = rclpy.duration.Duration(seconds=self.lqr_end.horizon / self.publish_rate)
        end_time = self.start_time + self.duration

        # Set up controls for feedback and analysis
        self.controls = [
            np.hstack((self.start_time.nanoseconds, self.xis[-1], self.us[-1], self.xi_g[-1], joint_positions, search_success, iterations, searches, residual))
        ]

        self.get_logger().info("Duration: {0} seconds".format(self.duration.nanoseconds / 1e9))

        # Loop over time
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        # Keep node alive until time is up
        while self.get_clock().now() < end_time:
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal cancelled")

                # Cancel the dynamic base frame broadcaster
                goal_handle.canceled()
                success = False
                break
            time.sleep(0.1)

        # Log going to cancel
        self.get_logger().info("Going to cancel")

        # Stop the timer
        self.timer.cancel()

        # Stop the dynamic base frame broadcaster
        cancel_future = self.dynamic_goal_future.result().cancel_goal_async()

        if self.controller_type == "DualLQR":
            # Combine two models
            lqr_list = [self.lqr_start, self.lqr_end]
            pickle.dump(
                lqr_list,
                open(
                    self.data_folder + "LQR_dual_startTime-{0}.pickle".format(self.start_time.nanoseconds),
                    "wb",
                ),
            )
        elif self.controller_type == "InfLQR":
            # Save model
            pickle.dump(
                self.store_model,
                open(
                    self.data_folder + "LQR_inf_startTime-{0}.pickle".format(self.start_time.nanoseconds),
                    "wb",
                ),
            )

        if success:
            self.get_logger().info("Completed successfully")
        else:
            self.get_logger().info("Unsuccessful")

        # Set result
        result_message = Float64MultiArray()

        # Combined trajectory elements into array
        combined_array = np.asarray(self.controls, dtype=np.float64)
        result_message.data = combined_array.flatten().tolist()
        result_message.layout.dim = [
            MultiArrayDimension(
                label="time",
                size=combined_array.shape[0],
                stride=combined_array.shape[0] * combined_array.shape[1],
            ),
            MultiArrayDimension(
                label="data",
                size=combined_array.shape[1],
                stride=combined_array.shape[1],
            ),
        ]

        # Set result
        result.performed_path = result_message

        self.get_logger().info("Publishing result")
        if success:
            goal_handle.succeed()
        return result


def main():
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()

    handle_trajectory_prediction = TrajectoryActionServer()

    executor.add_node(handle_trajectory_prediction)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
