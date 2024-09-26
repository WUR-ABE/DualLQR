#!/usr/bin/env python3

# Import modules
import numpy as np
import pickle
import transforms3d as t3d
import pbdlib as pbd
import sys
import time
import roboticstoolbox as rtb
from threading import Thread

import yaml
import os

# Locally import UR5e model
from rtb_model.UR5e import UR5e

# UR rtde imports
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from rtde_io import RTDEIOInterface as RTDEIO
import psutil

# Imports for natnetclient
import time
from natnet_client import DataDescriptions, DataFrame, NatNetClient

# Rel import
from pbdlib_custom import gmmr

# Fix the change in structure between pickle and ROS2 structure
sys.modules["gmmr"] = gmmr

class TrajectoryExecutorReactive(object):
    def __init__(self, sysArgs):
        # Helper variables
        self.publish_rate = 120.0
        self.controller_rate = 6.0
        self.controller_type = "DualLQR"
        self.data_folder = "!!FOLDER_TO_STORE_EXECUTED_TRAJECTORIES!!"
        self.model_name = "!!FOLDER_WITH_TRAINED_MODEL!!"
        self.rigid_bodies = None
        self.config_file = "calibration.yaml"
        self.reg_factor = -3.0
        self.transforms = []
        self.attached_suction = True

        # Parameters
        self.acc = 1.5
        self.vel = 3.0
        self.rtde_frequency = self.publish_rate
        self.dt = 1.0/self.publish_rate 
        self.flags = RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_USE_EXT_UR_CAP
        self.ur_cap_port = 50002
        self.robot_ip = "!!SET_YOUR_ROBOT_IP!!"

        self.lookahead_time = 0.03
        self.gain = 600

        # Define rotation between rtb_base_link and rtde_base_link
        self.tf_base_rtde_to_base_rtb = np.eye(4)
        self.tf_base_rtde_to_base_rtb[0:3, 0:3] = t3d.quaternions.quat2mat((0.0, 0.0, 0.0, -1.0))

        # Get calibration between OptiTrack and robot base
        self.get_optitrack_calibration()
        # Set up learned frames
        self.get_relaxed_frame()
        self.get_tcp_frame()
        self.get_cube_frame()
        self.get_base_link_frame()

        # Load UR5e model for IK solver
        self.ur5e = UR5e()
        self.ur5e_ets = self.ur5e.ets()

        # Set up UR rtde
        self.set_up_rtde()
        
        # Start up NatNetClient
        self.start_natnet()

        time.sleep(1)

        # Reset the robot to the home position for this task
        self.reset_manipulator()

        print("Starting up Trajectory Executor")
        self.intialize_control()

    def set_up_rtde(self):
        # ur_rtde realtime priorities
        rt_receive_priority = 90
        rt_control_priority = 85

        setup = False
        while not setup:
            try:
                self.rtde_r = RTDEReceive(self.robot_ip, self.rtde_frequency, [], True, False, rt_receive_priority)
                self.rtde_c = RTDEControl(self.robot_ip, self.rtde_frequency, self.flags, self.ur_cap_port, rt_control_priority)
                self.rtde_io = RTDEIO(self.robot_ip)
                setup = True
            except:
                print("Failed to setup RTDE, retrying in 1 second")
                time.sleep(1)

        # Set application real-time priority
        os_used = sys.platform
        process = psutil.Process(os.getpid())
        if os_used == "win32":  # Windows (either 32-bit or 64-bit)
            process.nice(psutil.REALTIME_PRIORITY_CLASS)
        elif os_used == "linux":  # linux
            rt_app_priority = 80
            param = os.sched_param(rt_app_priority)
            try:
                os.sched_setscheduler(0, os.SCHED_FIFO, param)
            except OSError:
                print("Failed to set real-time process scheduler to %u, priority %u" % (os.SCHED_FIFO, rt_app_priority))
            else:
                print("Process real-time priority set to: %u" % rt_app_priority)

    def get_optitrack_calibration(self):
        # Set path to config_file
        config_file_path = os.path.join(os.getcwd(), self.config_file)

        # Load yaml file
        with open(config_file_path) as file:
            calibration_dict = yaml.load(file, Loader=yaml.FullLoader)

        # Get calibration parameters
        calibration = calibration_dict["calibration"]

        # Build transformation matrix
        tf_plate_to_base = np.eye(4)
        tf_plate_to_base[0:3, 0:3] = t3d.quaternions.quat2mat((calibration["qw"], calibration["qx"], calibration["qy"], calibration["qz"], ))
        tf_plate_to_base[0:3, 3] = np.asarray((calibration["x"], calibration["y"], calibration["z"]))
        self.tf_plate_to_base_link = tf_plate_to_base

    def get_relaxed_frame(self):
        # Set transformation matrix
        tx = 0.181
        ty = -0.367
        tz = 0.615
        rx = 0.0
        ry = 0.0
        rz = 0.0
        rw = 1.0

        tf_base_link_to_relaxed = np.eye(4)
        tf_base_link_to_relaxed[0:3, 0:3] = t3d.quaternions.quat2mat((rw, rx, ry, rz))
        tf_base_link_to_relaxed[0:3, 3] = np.asarray((tx, ty, tz))

        self.tf_base_link_to_relaxed = tf_base_link_to_relaxed

    def get_base_link_frame(self):
        # Set transformation matrix
        tx = 0.0
        ty = 0.0
        tz = 0.0
        rx = 0.0
        ry = 0.0
        rz = 1.0
        rw = 0.0

        tf_base_to_base_link = np.eye(4)
        tf_base_to_base_link[0:3, 0:3] = t3d.quaternions.quat2mat((rw, rx, ry, rz))
        tf_base_to_base_link[0:3, 3] = np.asarray((tx, ty, tz))

        self.tf_base_to_base_link = tf_base_to_base_link
    
    def get_tcp_frame(self):
        # Set transformation matrix
        tx = 0.0
        ty = 0.0
        tz = 0.0
        rx = 0.0
        ry = 0.707
        rz = -0.707
        rw = 0.0

        tf_tcp_to_ee = np.eye(4)
        tf_tcp_to_ee[0:3, 0:3] = t3d.quaternions.quat2mat((rw, rx, ry, rz))
        tf_tcp_to_ee[0:3, 3] = np.asarray((tx, ty, tz))

        self.tf_tcp_to_ee = tf_tcp_to_ee

    def get_cube_frame(self):
        # Set transformation matrix
        tx = -0.23
        ty = 0.0
        tz = 0.0
        rx = 0.0
        ry = 0.0
        rz = 0.707
        rw = 0.707

        tf_cube_to_grasp_cube = np.eye(4)
        tf_cube_to_grasp_cube[0:3, 0:3] = t3d.quaternions.quat2mat((rw, rx, ry, rz))
        tf_cube_to_grasp_cube[0:3, 3] = np.asarray((tx, ty, tz))

        self.tf_cube_to_grasp_cube = tf_cube_to_grasp_cube

    def receive_new_frame(self, data_frame: DataFrame):
        # Get rigid bodies
        self.rigid_bodies = data_frame.rigid_bodies

        for rigid_body in data_frame.rigid_bodies:
            if rigid_body.id_num == 1007 and rigid_body.tracking_valid and not self.attached_suction:
                # Get position and rotation
                pos = rigid_body.pos
                rot = rigid_body.rot

                # Get transformation matrix
                rot_matrix = t3d.quaternions.quat2mat((rot[3], rot[0], rot[1], rot[2]))

                # Get transformation matrix
                matrix = np.eye(4)
                matrix[0:3, 0:3] = rot_matrix
                matrix[0:3, 3] = pos

                self.tf_world_to_cube = matrix
                if rigid_body.marker_error > 0.001: print("Cube error: ", rigid_body.marker_error)
            elif rigid_body.id_num == 1008 and rigid_body.tracking_valid and self.attached_suction:
                # Get position and rotation
                pos = rigid_body.pos
                rot = rigid_body.rot

                # Get transformation matrix
                rot_matrix = t3d.quaternions.quat2mat((rot[3], rot[0], rot[1], rot[2]))

                # Get transformation matrix
                matrix = np.eye(4)
                matrix[0:3, 0:3] = rot_matrix
                matrix[0:3, 3] = pos

                self.tf_world_to_cube = matrix
                if rigid_body.marker_error > 0.001: print("Apple error: ", rigid_body.marker_error)
            elif rigid_body.id_num == 1005 and rigid_body.tracking_valid:
                # Get position and rotation
                pos = rigid_body.pos
                rot = rigid_body.rot

                # Get transformation matrix
                rot_matrix = t3d.quaternions.quat2mat((rot[3], rot[0], rot[1], rot[2]))

                # Get transformation matrix
                matrix = np.eye(4)
                matrix[0:3, 0:3] = rot_matrix
                matrix[0:3, 3] = pos

                # Multiply with plate to base transformation
                self.tf_world_to_plate = matrix
                if rigid_body.marker_error > 0.001: print("Plate error: ", rigid_body.marker_error)
        
        if self.tf_world_to_cube is not None and self.tf_world_to_plate is not None:
            # Calculate transformation matrix between base link and cube
            self.tf_base_link_to_grasp_cube = np.linalg.multi_dot([np.linalg.inv(self.tf_plate_to_base_link), np.linalg.inv(self.tf_world_to_plate), self.tf_world_to_cube, self.tf_cube_to_grasp_cube])

        # Start up logging
        self.transform_logger()
        
    def receive_new_desc(self, desc: DataDescriptions):
        print("Received data descriptions.")

    def start_natnet(self):
        self.streaming_client = NatNetClient(server_ip_address="192.168.10.1", local_ip_address="192.168.10.70", use_multicast=True)
        self.streaming_client.on_data_description_received_event.handlers.append(self.receive_new_desc)
        self.streaming_client.on_data_frame_received_event.handlers.append(self.receive_new_frame)

        self.streaming_client.connect()
        self.streaming_client.request_modeldef()
        self.streaming_client.run_async()

    def matrix_to_pose(self, matrix, axes="sxyz"):
        x, y, z = matrix[0:3, 3]
        roll, pitch, yaw = t3d.euler.mat2euler(matrix[0:3, 0:3], axes=axes)
        pose = np.asarray((x, y, z, roll, pitch, yaw))
        return pose

    def transform_control(self, control, matrix):
        # Transform position vector to robot base frame
        pos_vec = control[0:3].reshape((3, 1))
        pos_tvec = np.dot(matrix[0:3, 0:3], pos_vec)

        # Transform orientation vector to robot base frame
        rot_vec = control[3:6].reshape((3, 1))
        rot_tvec = np.dot(matrix[0:3, 0:3], rot_vec)

        # Combine position and orientation
        transformed_control = np.vstack((pos_tvec, rot_tvec)).flatten()
        return transformed_control

    def reset_manipulator(self):
        # Determine target tcp, which is relaxed frame for ee
        target_tcp = np.linalg.multi_dot([self.tf_base_link_to_relaxed, np.linalg.inv(self.tf_tcp_to_ee)])
        
        # Calculate IK
        joint_positions, search_success, iterations, searches, residual = self.ur5e_ets.ik_LM(
            target_tcp,
            q0=self.rtde_r.getActualQ(),
            ilimit=30,  # Maximum iterations allowed in a search for a solution
            slimit=100,  # Maximum searches allowed for a solution
            tol=1e-6,  # Solution tolerance
            k=0.1,
            joint_limits=False,
            method="chan",
        )

        self.rtde_c.moveJ(joint_positions)

        # Open gripper
        if self.attached_2f:
            self.gripper.goto(pos=0.14)

        return True

    def array_to_matrix(self, vector, axes="sxyz"):
        # Get tranformation matrix from array
        rot_array = t3d.euler.euler2mat(vector[3], vector[4], vector[5], axes=axes)
        tf = np.eye(4)
        tf[0:3, 0:3] = rot_array
        tf[0:3, 3] = vector[0:3]
        return tf

    def transform_logger(self):
        # Log transform between base and dynamic frame
        try:
            tf_relaxed_frame_to_grasp_cube = np.linalg.multi_dot([np.linalg.inv(self.tf_base_link_to_relaxed), self.tf_base_link_to_grasp_cube])
            quaternion_relaxed_frame_to_grasp_cube = t3d.quaternions.mat2quat(tf_relaxed_frame_to_grasp_cube[0:3, 0:3])
            tf_relaxed_frame_to_ee = np.linalg.multi_dot([np.linalg.inv(self.tf_base_link_to_relaxed), np.asarray(self.ur5e_ets.fkine(self.rtde_r.getActualQ())), self.tf_tcp_to_ee])
            quaternion_relaxed_frame_to_ee = t3d.quaternions.mat2quat(tf_relaxed_frame_to_ee[0:3, 0:3])
            self.transforms += [
                [
                    time.time(),
                    tf_relaxed_frame_to_grasp_cube[0, 3],
                    tf_relaxed_frame_to_grasp_cube[1, 3],
                    tf_relaxed_frame_to_grasp_cube[2, 3],
                    quaternion_relaxed_frame_to_grasp_cube[1],
                    quaternion_relaxed_frame_to_grasp_cube[2],
                    quaternion_relaxed_frame_to_grasp_cube[3],
                    quaternion_relaxed_frame_to_grasp_cube[0],
                    time.time(),
                    tf_relaxed_frame_to_ee[0, 3],
                    tf_relaxed_frame_to_ee[1, 3],
                    tf_relaxed_frame_to_ee[2, 3],
                    quaternion_relaxed_frame_to_ee[1],
                    quaternion_relaxed_frame_to_ee[2],
                    quaternion_relaxed_frame_to_ee[3],
                    quaternion_relaxed_frame_to_ee[0],
                ]
            ]
        except:
            pass

    def timer_callback(self):
        # Time since started
        current_time = time.time()

        diff = current_time - self.start_time

        # Fraction of time
        time_fract = diff / self.duration

        # Integer time step
        t = int(time_fract * (self.lqr_end.horizon - 2))

        if t > self.lqr_end.horizon - 2:
            print("Time step too large")  
            t = self.lqr_end.horizon - 2

        # Get current state
        # Calculate forward kinematics
        tf_base_to_tcp = np.asarray(self.ur5e_ets.fkine(self.rtde_r.getActualQ()))
        # Calculate matrix between ee and moving frame
        tf_moving_frame_to_ee = np.linalg.multi_dot([self.tf_moving_frame_to_grasp_cube, np.linalg.inv(self.tf_base_link_to_grasp_cube), tf_base_to_tcp, self.tf_tcp_to_ee])
        # Calculate pose
        pose_i = self.matrix_to_pose(tf_moving_frame_to_ee)
        self.xis += [pose_i] # [self.xi_g[-1][:-1]] # 

        if self.controller_type == "DualLQR":
            # Get current state in start frame
            tf_ee_to_relaxed_frame = np.linalg.inv(np.linalg.multi_dot([np.linalg.inv(self.tf_tcp_to_ee), np.linalg.inv(tf_base_to_tcp), self.tf_base_link_to_relaxed]))
            pose_i_start = self.matrix_to_pose(tf_ee_to_relaxed_frame)

            # Determine both control inputs
            self.us_start += [-self.lqr_start._K[t].dot(pose_i_start) + self.lqr_start._Kv[t].dot(self.lqr_start._v[t + 1])]
            self.us_end += [-self.lqr_end._K[t].dot(self.xis[-1]) + self.lqr_end._Kv[t].dot(self.lqr_end._v[t + 1])]

            # Get rotation matrix between robot base frame and relaxed_ik frame
            tf_moving_frame_to_relaxed_frame = np.linalg.multi_dot([self.tf_moving_frame_to_grasp_cube, np.linalg.inv(self.tf_base_link_to_grasp_cube), self.tf_base_link_to_relaxed])
            transformed_us_start = self.transform_control(self.us_start[-1], tf_moving_frame_to_relaxed_frame)

            # Get both inverse covariance matrices
            lambda_start, _ = self.lqr_start.get_Q_z(t)
            lambda_end, _ = self.lqr_end.get_Q_z(t)

            # Make everything but diagonal zero
            lambda_start = np.diag(np.diag(lambda_start))
            lambda_end = np.diag(np.diag(lambda_end))

            # Compute product of both control inputs
            weighted_us = np.linalg.inv(lambda_start + lambda_end).dot(lambda_start.dot(transformed_us_start) + lambda_end.dot(self.us_end[-1]))
            self.us += [weighted_us]
        else:
            raise ValueError("Controller type not supported")

        # Goal pose, based on current pose and control input based on current pose
        # Add time fraction to goal pose
        self.xi_g += [
            np.append(
                self.lqr_end.A.dot(self.xis[-1]) + self.lqr_end.B.dot(self.us[-1]),
                time_fract,
            )
        ]  

        # Determine goal pose from robot base frame to tcp
        goal_moving_frame_to_ee = self.array_to_matrix(
            self.xi_g[-1][:-1],
        )
        # Calculate goal pose in robot base frame
        goal_base_to_tcp = np.linalg.multi_dot([self.tf_base_link_to_grasp_cube, np.linalg.inv(self.tf_moving_frame_to_grasp_cube), goal_moving_frame_to_ee, np.linalg.inv(self.tf_tcp_to_ee)])

        # Use TracIK to solve IK
        joint_positions, search_success, iterations, searches, residual = self.ur5e_ets.ik_LM(
            goal_base_to_tcp,
            q0=self.rtde_r.getActualQ(),
            ilimit=30,  # Maximum iterations allowed in a search for a solution
            slimit=100,  # Maximum searches allowed for a solution
            tol=1e-6,  # Solution tolerance
            k=0.1,
            joint_limits=False,
            method="chan",
        )

        chamber_pressure = self.rtde_r.getStandardAnalogInput0()

        if chamber_pressure < 1.0:
            self.rtde_c.servoJ(joint_positions, self.vel, self.acc, self.dt, self.lookahead_time, self.gain)

        # Determine goal in relaxed frame
        goal_relaxed_frame_to_ee = np.linalg.multi_dot([np.linalg.inv(self.tf_base_link_to_relaxed), goal_base_to_tcp, self.tf_tcp_to_ee])
        goal_pose = self.matrix_to_pose(goal_relaxed_frame_to_ee)

        # Store controls
        self.controls += [
            np.hstack((current_time, time.time(), self.xis[-1], self.us[-1], self.xi_g[-1], joint_positions, search_success, iterations, searches, residual, goal_pose, chamber_pressure))
        ]

    def intialize_control(self):

        # Set success boolean
        success = True

        # Calculate relative transform between object and relaxed frame, this will track the object
        self.tf_moving_frame_to_grasp_cube = np.linalg.multi_dot([np.linalg.inv(self.tf_base_link_to_relaxed), self.tf_base_link_to_grasp_cube])

        # Get tcp
        joint_positions = self.rtde_r.getActualQ()
        # Calculate forward kinematics
        tf_base_to_tcp = np.asarray(self.ur5e_ets.fkine(joint_positions))
        
        # Get tf between ee and moving_frame, going through grasp_cube
        tf_moving_frame_to_ee = np.linalg.multi_dot([self.tf_moving_frame_to_grasp_cube, np.linalg.inv(self.tf_base_link_to_grasp_cube), tf_base_to_tcp, self.tf_tcp_to_ee])
        # Calculate pose
        pose_0 = self.matrix_to_pose(tf_moving_frame_to_ee)
        print("Pose 0: \n {0}".format(pose_0))

        self.initial_pose = pose_0

        # Get transform between base and goal
        pose_n = self.matrix_to_pose(self.tf_moving_frame_to_grasp_cube)
        print("Pose n: \n {0}".format(pose_n))

        # Load model from file location
        self.model = pickle.load(open(self.model_name, "rb"), encoding="latin1")

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
            self.lqr_start.gmm_u = self.reg_factor

            self.lqr_end = pbd.LQR(A, b, dt=1.0 / self.publish_rate, horizon=len(mu2))
            self.lqr_end.gmm_xi = [mu2[:, 1:], sigma2[:, 1:, 1:], self.sq]
            self.lqr_end.gmm_u = self.reg_factor

            # Fit LQR
            self.lqr_start.ricatti()
            self.lqr_end.ricatti()

            # Initialize first control input
            self.us_start = [-self.lqr_start._K[0].dot(pose_0) + self.lqr_start._Kv[0].dot(self.lqr_start._v[0])]
            self.us_end = [-self.lqr_end._K[0].dot(pose_0) + self.lqr_end._Kv[0].dot(self.lqr_end._v[0])]
        else:
            self.get_logger().error("Invalid controller type")
            return False

        # Release io pin DO0
        self.rtde_io.setStandardDigitalOut(0, False)

        # Wait for 3 seconds to get rid of initial noise
        time.sleep(3)

        # Get current time
        self.start_time = time.time()
        self.duration = float(self.lqr_end.horizon) / self.publish_rate
        end_time = self.start_time + self.duration

        # Set up controls for feedback and analysis
        self.controls = [
            np.hstack((self.start_time, self.start_time, self.xis[-1], self.us[-1], self.xi_g[-1], joint_positions, 1, 0, 1, 0.0, pose_0, 0.0))
        ]

        try:
            while time.time() < end_time:
                t_start = self.rtde_c.initPeriod()
                self.timer_callback()
                self.rtde_c.waitPeriod(t_start)
        except KeyboardInterrupt:
            print("Control Interrupted!")

        # Send zero velocity to stop the robot
        self.rtde_c.servoStop()
        self.rtde_c.stopScript()
        self.streaming_client.shutdown()

        if self.controller_type == "DualLQR":
            # Combine two models
            lqr_list = [self.lqr_start, self.lqr_end]
            pickle.dump(
                lqr_list,
                open(
                    self.data_folder + "LQR_dual_startTime-{0}.pickle".format(self.start_time),
                    "wb",
                ),
            )

        # Combined trajectory elements into array
        all_data = [np.asarray(self.controls, dtype=np.float64), np.asarray(self.transforms)]

        # Save data
        pickle.dump(
            all_data,
            open(
                self.data_folder + "result_regFactor-{0}_controllerType-{1}_controllerRate-{2}_lookaheadTime-{3}_gain-{4}_startTime-{5}.pickle".format(
                    self.reg_factor,
                    self.controller_type,
                    self.publish_rate,
                    self.lookahead_time,
                    self.gain,
                    self.start_time,
                ),
                "wb",
            ),
        )
        
        # Activate io pin DO0
        self.rtde_io.setStandardDigitalOut(0, True)

        if self.attached_2f:
            self.running_gripper = False
            self.thread_gripper.join()

        return success

if __name__ == "__main__":
    TrajectoryExecutorReactive(sys.argv[1:])