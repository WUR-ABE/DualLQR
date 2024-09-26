import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
import time
import numpy as np
import pickle
import subprocess
import os

from lfd_executor_msgs.action import TrajectoryExecuteReactive, ControlledGoalPose
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped

# TF subscriber imports
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class SimulationTester(Node):
    def __init__(self):
        super().__init__("simulation_testing")

        # Get params
        self.declare_parameter(
            "noise_type",
            "sinus",
            ParameterDescriptor(description="type of noise, can be none, step, or sinus"),
        )
        self.declare_parameter(
            "noise_dimension",
            "yaw",
            ParameterDescriptor(description="dimension of noise, can be x, y, z, roll, pitch, or yaw"),
        )
        self.declare_parameter(
            "noise_magnitude",
            0.05,
            ParameterDescriptor(description="magnitude/amplitude of noise"),
        )
        self.declare_parameter(
            "noise_time",
            2.0,
            ParameterDescriptor(description="steps: time at which noise is applied, sinus: frequency of noise"),
        )
        self.declare_parameter(
            "reg_factor",
            0.0,
            ParameterDescriptor(description="regularization factor for LQR in reactive TP-GMR"),
        )
        self.declare_parameter(
            "x",
            0.2171284883,
            ParameterDescriptor(description="x position of goal pose"),
        )
        self.declare_parameter(
            "y",
            0.2657070818,
            ParameterDescriptor(description="y position of goal pose"),
        )
        self.declare_parameter(
            "z",
            -0.2620939713,
            ParameterDescriptor(description="z position of goal pose"),
        )
        self.declare_parameter(
            "qx",
            -0.0005375944,
            ParameterDescriptor(description="quaternion x orientation of goal pose"),
        )
        self.declare_parameter(
            "qy",
            0.0041282719,
            ParameterDescriptor(description="quaternion y orientation of goal pose"),
        )
        self.declare_parameter(
            "qz",
            0.6680464824,
            ParameterDescriptor(description="quaternion z orientation of goal pose"),
        )
        self.declare_parameter(
            "qw",
            0.7441078992,
            ParameterDescriptor(description="quaternion w orientation of goal pose"),
        )
        self.declare_parameter(
            "publish_rate",
            125.0,
            ParameterDescriptor(description="publish rate of goal pose broadcaster"),
        )
        self.declare_parameter(
            "controller_type",
            "SingleLQR",
            ParameterDescriptor(description="Should match the value of the same parameter in the reactive TP-GMR node"),
        )
        self.declare_parameter(
            "repetition",
            "DefaultGoal",
            ParameterDescriptor(description="Describer of the pose repetition"),
        )
        self.declare_parameter(
            "data_folder",
            "/media/data/OneDrive",
            ParameterDescriptor(description="Path to the data folder"),
        )

        # Helper variables
        self.noise_type = self.get_parameter("noise_type").get_parameter_value().string_value
        self.noise_dimension = self.get_parameter("noise_dimension").get_parameter_value().string_value
        self.noise_magnitude = self.get_parameter("noise_magnitude").get_parameter_value().double_value
        self.noise_time = self.get_parameter("noise_time").get_parameter_value().double_value
        self.reg_factor = self.get_parameter("reg_factor").get_parameter_value().double_value
        self.x = self.get_parameter("x").get_parameter_value().double_value
        self.y = self.get_parameter("y").get_parameter_value().double_value
        self.z = self.get_parameter("z").get_parameter_value().double_value
        self.qx = self.get_parameter("qx").get_parameter_value().double_value
        self.qy = self.get_parameter("qy").get_parameter_value().double_value
        self.qz = self.get_parameter("qz").get_parameter_value().double_value
        self.qw = self.get_parameter("qw").get_parameter_value().double_value
        self.publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        self.controller_type = self.get_parameter("controller_type").get_parameter_value().string_value
        self.repetition = self.get_parameter("repetition").get_parameter_value().string_value
        self.data_folder = self.get_parameter("data_folder").get_parameter_value().string_value

        self.model_file = os.path.join(self.data_folder, "MSc thesis/05 Experiments/apple_harvesting/models/approach/gmr/demos_40.pickle")

        # Set default callback to allow for multithreading TF
        self._default_callback_group = ReentrantCallbackGroup()

        # Set up action client for reactive TP-GMR
        self._action_client_tpgmr = ActionClient(self, TrajectoryExecuteReactive, "action_traj_exec_react")

        # Set up action client for controlled goal pose broadcaster
        self._action_client_goal_pose = ActionClient(self, ControlledGoalPose, "action_controlled_goal_pose")

        # Set up publisher for resetting manipulator
        self.publisher = self.create_publisher(Float64MultiArray, "/forward_position_controller/commands", 10)

        # Create TF subscribers
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Check if forward_position_controller is loaded
        while "forward_position_controller" not in str(subprocess.Popen(["ros2", "control", "list_controllers"], stdout=subprocess.PIPE).communicate()[0]):
            load_pos_controller = subprocess.Popen(
                [
                    "ros2",
                    "control",
                    "switch_controllers",
                    "--configure",
                    "forward_position_controller",
                ],
                stdout=subprocess.PIPE,
            )

            time.sleep(0.5)

        # Run the tester function
        self.simulation_tester()

    def reset_manipulator(self):
        # Reset manipulator
        msg = Float64MultiArray()
        msg.data = [-0.6324, -1.7991, 1.8902, -3.2281, -2.5102, 0.0066]

        self.publisher.publish(msg)

    def send_goal_pose_broadcaster_goal(self, transformation, noise_type, noise_dimension, noise_magnitude, noise_rate):
        goal_msg = ControlledGoalPose.Goal()
        goal_msg.noise_type = noise_type
        goal_msg.noise_dimension = noise_dimension
        goal_msg.noise_level = noise_magnitude
        goal_msg.noise_rate = noise_rate
        goal_msg.transformation = transformation

        self._action_client_goal_pose.wait_for_server()

        return self._action_client_goal_pose.send_goal_async(goal_msg)

    def send_tpgmr_goal(self, reg_factor):
        goal_msg = TrajectoryExecuteReactive.Goal()
        goal_msg.reg_factor = reg_factor

        goal_msg.model_name = self.model_file

        self._action_client_tpgmr.wait_for_server()

        return self._action_client_tpgmr.send_goal_async(goal_msg)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.result = future.result().result

    def transform_logger(self):
        # Log transform between base and dynamic frame
        try:
            transform_cube = self.tf_buffer.lookup_transform("relaxed_ik", "cube_ik", rclpy.time.Time())
            transform_tool = self.tf_buffer.lookup_transform("relaxed_ik", "tool_ik", rclpy.time.Time())
            self.transforms += [
                [
                    transform_cube.header.stamp.nanosec + transform_cube.header.stamp.sec * 1e9,
                    transform_cube.transform.translation.x,
                    transform_cube.transform.translation.y,
                    transform_cube.transform.translation.z,
                    transform_cube.transform.rotation.x,
                    transform_cube.transform.rotation.y,
                    transform_cube.transform.rotation.z,
                    transform_cube.transform.rotation.w,
                    transform_tool.header.stamp.nanosec + transform_tool.header.stamp.sec * 1e9,
                    transform_tool.transform.translation.x,
                    transform_tool.transform.translation.y,
                    transform_tool.transform.translation.z,
                    transform_tool.transform.rotation.x,
                    transform_tool.transform.rotation.y,
                    transform_tool.transform.rotation.z,
                    transform_tool.transform.rotation.w,
                ]
            ]
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass

    def simulation_tester(self):
        # Move manipulator to starting position
        self.reset_manipulator()
        time.sleep(3.0)

        # Set up transformation to broadcast
        transformation = TransformStamped()
        transformation.transform.translation.x = self.x
        transformation.transform.translation.y = self.y
        transformation.transform.translation.z = self.z
        transformation.transform.rotation.x = self.qx
        transformation.transform.rotation.y = self.qy
        transformation.transform.rotation.z = self.qz
        transformation.transform.rotation.w = self.qw
        transformation.header.frame_id = "relaxed_ik"
        transformation.child_frame_id = "cube_ik"

        # Start broadcasting the goal pose
        goal_pose_broadcaster_future = self.send_goal_pose_broadcaster_goal(
            transformation,
            self.noise_type,
            self.noise_dimension,
            self.noise_magnitude,
            self.noise_time,
        )

        # Wait for the goal pose broadcaster to publish the frame
        transform_waiter = self.tf_buffer.wait_for_transform_async("cube_ik", "relaxed_ik", time=rclpy.time.Time())
        while not transform_waiter.done():
            time.sleep(0.2)
        self.get_logger().info("Goal frame published")

        # Start reactive TP-GMR
        reactive_tpgmr_future = self.send_tpgmr_goal(self.reg_factor)

        # Start logging the transform between base and dynamic frame
        self.get_logger().info("Starting logger")
        self.transforms = []
        transform_logger_timer = self.create_timer(1.0 / self.publish_rate, self.transform_logger)

        # Add done callback to reactive TP-GMR
        reactive_tpgmr_future.add_done_callback(self.goal_response_callback)

        # Wait for the reactive TP-GMR to finish
        self.get_logger().info("Waiting for reactive TP-GMR to finish")
        self.result = None
        self.transforms = []

        try:
            while rclpy.ok():
                if self.result:
                    # Cancel the goal pose broadcaster
                    cancel_goal_pose = goal_pose_broadcaster_future.result().cancel_goal_async()
                    transform_logger_timer.cancel()
                    break
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.get_logger().info("Caught keyboard interrupt, I have no purpose...")
            # Cancel the goal pose broadcaster
            cancel_goal_pose = goal_pose_broadcaster_future.result().cancel_goal_async()
            transform_logger_timer.cancel()
            # Cancel the reactive TP-GMR
            cancel_reactive_tpgmr = reactive_tpgmr_future.result().cancel_goal_async()

        dims = tuple(map(lambda x: x.size, self.result.performed_path.layout.dim))
        np_result = np.array(self.result.performed_path.data, dtype=float).reshape(dims).astype(np.float64)

        # Store result
        if self.noise_dimension == "yaxis":
            noise_dimension_write = "y"
        else:
            noise_dimension_write = self.noise_dimension

        all_data = [np_result, np.asarray(self.transforms)]
        pickle.dump(
            all_data,
            open(
                self.data_folder
                + "PhD/03 Study 2/03 Software/trajectories/result_noiseType-{0}_noiseDimension-{1}_noiseMagnitude-{2}_noiseRate-{3}_regFactor-{4}_controllerType-{5}_controllerRate-{6}_repetition-{7}_startTime-{8}.pickle".format(
                    self.noise_type,
                    noise_dimension_write,
                    self.noise_magnitude,
                    self.noise_time,
                    self.reg_factor,
                    self.controller_type,
                    self.publish_rate,
                    self.repetition,
                    self.transforms[0][0],
                ),
                "wb",
            ),
        )

        # End this node when done with loop
        self.get_logger().warn("Test finished")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()

    simulation_tester = SimulationTester()

    executor.add_node(simulation_tester)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
