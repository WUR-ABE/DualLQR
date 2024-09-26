import subprocess
import signal
import time
import transforms3d as tf3d
import os

start_time = time.time()


def kill_ros_nodes(exceptions=[], depth=0):
    depth += 1
    # Append empty string to exceptions
    actual_exceptions = exceptions + [""]
    # List ros nodes still running
    ros2_node_list = subprocess.run(["ros2", "node", "list"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    running_nodes = ros2_node_list.stdout.decode("utf-8").split("\n")
    kill_nodes = list(set(running_nodes) - set(actual_exceptions))

    # Shut down all remaining nodes
    for node in kill_nodes:
        if node != "":
            if depth > 3:
                if "controller_manager" in node:
                    subprocess.run(["killall", "-9", "ros2_control_node"])
                elif "spawner" in node:
                    subprocess.run(["killall", "-9", "spawner"])
                else:
                    subprocess.run(["killall", "-9", node[1:]])
            else:
                if "controller_manager" in node:
                    subprocess.run(["killall", "ros2_control_node"])
                elif "spawner" in node:
                    subprocess.run(["killall", "spawner"])
                else:
                    subprocess.run(["killall", node[1:]])

    # Wait for all nodes to terminate
    time.sleep(1)

    # Check if all nodes have been terminated
    ros2_node_list = subprocess.run(["ros2", "node", "list"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    running_nodes = ros2_node_list.stdout.decode("utf-8").split("\n")
    kill_nodes = list(set(running_nodes) - set(actual_exceptions))
    if len(kill_nodes) > 0:
        print("Could not kill all nodes, trying again")
        print("Nodes still running: " + str(kill_nodes))
        kill_ros_nodes(exceptions=exceptions, depth=depth)


def launch_fake_ur5e():
    # Recursive function to launch the UR5e

    # Launch fake UR5e
    fake_ur5e = subprocess.Popen(
        [
            "ros2",
            "launch",
            "reactive_arm_control",
            "reactive_arm_control.launch.py",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    # Wait for fake UR5e to start
    time.sleep(5)

    # Check the stdout for the string "Warning: "
    output_line = fake_ur5e.stdout.readline().decode("utf-8")
    if output_line != "":
        if "Warning: " in output_line:
            print("Warning received, repeating run")
            # Kill processes if warning is detected
            fake_ur5e.send_signal(signal.SIGINT)
            # Print stdout of lfd execution
            print("Warning: " + output_line)
            time.sleep(1.0)
            # Repeat function
            fake_ur5e = launch_fake_ur5e()

    return fake_ur5e


# Goal poses
goal_poses = [
    {
        "description": "DefaultGoal",
        "x": "0.2171284883",
        "y": "0.2657070818",
        "z": "-0.2620939713",
        "roll": "0.00471583",
        "pitch": "0.00686209",
        "yaw": "1.46319256",
    },
    {
        "description": "RollPlusGoal",
        "x": "0.2171284883",
        "y": "0.2657070818",
        "z": "-0.2620939713",
        "roll": "0.20471583",
        "pitch": "0.00686209",
        "yaw": "1.46319256",
    },
    {
        "description": "RollMinusGoal",
        "x": "0.2171284883",
        "y": "0.2657070818",
        "z": "-0.2620939713",
        "roll": "-0.19528417",
        "pitch": "0.00686209",
        "yaw": "1.46319256",
    },
    {
        "description": "YawPlusGoal",
        "x": "0.2171284883",
        "y": "0.2657070818",
        "z": "-0.2620939713",
        "roll": "0.00471583",
        "pitch": "0.00686209",
        "yaw": "1.66319256",
    },
    {
        "description": "YawMinusGoal",
        "x": "0.2171284883",
        "y": "0.2657070818",
        "z": "-0.2620939713",
        "roll": "0.00471583",
        "pitch": "0.00686209",
        "yaw": "1.26319256",
    },
    {
        "description": "XPlusGoal",
        "x": "0.3171284883",
        "y": "0.2657070818",
        "z": "-0.2620939713",
        "roll": "0.00471583",
        "pitch": "0.00686209",
        "yaw": "1.46319256",
    },
    {
        "description": "XMinusGoal",
        "x": "0.1171284883",
        "y": "0.2657070818",
        "z": "-0.2620939713",
        "roll": "0.00471583",
        "pitch": "0.00686209",
        "yaw": "1.46319256",
    },
    {
        "description": "YPlusGoal",
        "x": "0.2171284883",
        "y": "0.4657070818",
        "z": "-0.2620939713",
        "roll": "0.00471583",
        "pitch": "0.00686209",
        "yaw": "1.46319256",
    },
    {
        "description": "YMinusGoal",
        "x": "0.2171284883",
        "y": "0.0657070818",
        "z": "-0.2620939713",
        "roll": "0.00471583",
        "pitch": "0.00686209",
        "yaw": "1.46319256",
    },
    {
        "description": "ZPlusGoal",
        "x": "0.2171284883",
        "y": "0.2657070818",
        "z": "-0.2120939713",
        "roll": "0.00471583",
        "pitch": "0.00686209",
        "yaw": "1.46319256",
    },
    {
        "description": "ZMinusGoal",
        "x": "0.2171284883",
        "y": "0.2657070818",
        "z": "-0.3120939713",
        "roll": "0.00471583",
        "pitch": "0.00686209",
        "yaw": "1.46319256",
    },
]

# Testing all parameters
testing_dict = {
    "sinus": [
        {
            "dims": ["x", "y", "z"],
            "mags": [
                "0.1",
                "0.15",
                "0.05",
            ],
            "time": ["1.0"],
            "regu": [
                "-3.0",
                "-2.7",
                "-2.4",
                "-2.1",
                "-1.8",
                "-1.5",
                "-1.2",
                "-0.9",
                "-0.6",
                "-0.3",
                "0.0",
                "0.3",
                "0.6",
                "0.9",
                "1.2",
                "1.5",
                "1.8",
                "2.1",
                "2.4",
                "2.7",
                "3.0",
            ],
            "controller_type": [
                "InfLQR", "DualLQR",
            ],
        },
        {
            "dims": ["roll", "pitch", "yaw"],
            "mags": [
                "0.3",
                "0.45",
                "0.15",
            ],
            "time": ["1.0"],
            "regu": [
                "-3.0",
                "-2.7",
                "-2.4",
                "-2.1",
                "-1.8",
                "-1.5",
                "-1.2",
                "-0.9",
                "-0.6",
                "-0.3",
                "0.0",
                "0.3",
                "0.6",
                "0.9",
                "1.2",
                "1.5",
                "1.8",
                "2.1",
                "2.4",
                "2.7",
                "3.0",
            ],
            "controller_type": [
                "InfLQR", "DualLQR",
            ],
        },
    ],
    "none": [
        {
            "regu": [
                "-3.0",
                "-2.7",
                "-2.4",
                "-2.1",
                "-1.8",
                "-1.5",
                "-1.2",
                "-0.9",
                "-0.6",
                "-0.3",
                "0.0",
                "0.3",
                "0.6",
                "0.9",
                "1.2",
                "1.5",
                "1.8",
                "2.1",
                "2.4",
                "2.7",
                "3.0",
            ],
            "controller_type": [
                "InfLQR", "DualLQR",
            ],
        }
    ],
}

repetitions = len(goal_poses)

# Frequency of LfD algorithm & moving goal pose broadcaster
control_rate = "6.0"
logging_rate = "30.0"

# List ros nodes running, which need to be kept alive for the entire loop
ros2_node_list = subprocess.run(["ros2", "node", "list"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
durable_nodes = ros2_node_list.stdout.decode("utf-8").split("\n")

# Define model file path
data_folder = "!!SET_LOCATION_OF_DATA_FOLDER!!"

# Get list of files containing the performed tests
trajectories_path = os.path.join(data_folder, "!!SET_LOCATION_OF_TRAJECTORIES_FOLDER!!")
trajectory_files = next(os.walk(trajectories_path))[2]

# Select only the files containing the performed tests, starting with "result"
trajectory_files = [file for file in trajectory_files if file.startswith("result")]

try:
    for noise_type in testing_dict.keys():
        for _, key_dict in enumerate(testing_dict[noise_type]):
            if len(key_dict.keys()) == 2:
                dims = ["x"]
                mags = ["0.0"]
                times = ["0.5"]
                regus = key_dict["regu"]
                controller_types = key_dict["controller_type"]
            else:
                dims = key_dict["dims"]
                mags = key_dict["mags"]
                times = key_dict["time"]
                regus = key_dict["regu"]
                controller_types = key_dict["controller_type"]

            for noise_dimension in dims:
                for noise_magnitude in mags:
                    for noise_time in times:
                        for reg_factor in regus:
                            for controller_type in controller_types:
                                ## Check how many runs with this noise type have already been done
                                # Build the result string
                                result_string = (
                                    "result_noiseType-{0}_noiseDimension-{1}_noiseMagnitude-{2}_noiseRate-{3}_regFactor-{4}_controllerType-{5}".format(
                                        noise_type,
                                        noise_dimension,
                                        noise_magnitude,
                                        noise_time,
                                        reg_factor,
                                        controller_type,
                                    )
                                )
                                # Check if this result string is already in the list of performed tests
                                current_trajectory_files = next(os.walk(trajectories_path))[2]
                                j = sum([True for file in current_trajectory_files if file.startswith(result_string)])

                                if j >= repetitions:
                                    print("Already performed test: " + result_string)
                                    continue

                                # Launch fake UR5e
                                fake_ur5e = launch_fake_ur5e()

                                # List UR5e nodes
                                ros2_node_list = subprocess.run(
                                    ["ros2", "node", "list"],
                                    stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE,
                                )
                                ur5e_nodes = ros2_node_list.stdout.decode("utf-8").split("\n")

                                print(
                                    "Running with noise type {0}, noise dimension {1}, noise magnitude {2}, noise time {3}, reg factor {4}, controller type {5}".format(
                                        noise_type,
                                        noise_dimension,
                                        noise_magnitude,
                                        noise_time,
                                        reg_factor,
                                        controller_type,
                                    )
                                )

                                i = 0

                                while i < repetitions:
                                    # Get goal pose for specific repetition
                                    pose_dict = goal_poses[i]
                                    # Check result for specific repetition
                                    result_string = "result_noiseType-{0}_noiseDimension-{1}_noiseMagnitude-{2}_noiseRate-{3}_regFactor-{4}_controllerType-{5}_controllerRate-{6}_repetition-{7}".format(
                                        noise_type,
                                        noise_dimension,
                                        noise_magnitude,
                                        noise_time,
                                        reg_factor,
                                        controller_type,
                                        logging_rate,
                                        pose_dict["description"],
                                    )
                                    # Check if this result string is already in the list of performed tests
                                    current_trajectory_files = next(os.walk(trajectories_path))[2]
                                    peformed_check = sum([True for file in current_trajectory_files if file.startswith(result_string)])
                                    if peformed_check > 0:
                                        i += 1
                                        continue

                                    if noise_dimension == "y":
                                        noise_dim = "yaxis"
                                    else:
                                        noise_dim = noise_dimension
                                    # Launch relaxed frame broadcaster and tcp for ik broadcaster
                                    relaxed_frame_broadcaster = subprocess.Popen(
                                        [
                                            "ros2",
                                            "run",
                                            "lfd_executor",
                                            "relaxed_frame_broadcaster",
                                        ],
                                        stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE,
                                    )
                                    tcp_for_ik_broadcaster = subprocess.Popen(
                                        [
                                            "ros2",
                                            "run",
                                            "lfd_executor",
                                            "tcp_for_ik_broadcaster",
                                        ],
                                        stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE,
                                    )

                                    x = pose_dict["x"]
                                    y = pose_dict["y"]
                                    z = pose_dict["z"]
                                    quat = tf3d.euler.euler2quat(
                                        float(pose_dict["roll"]),
                                        float(pose_dict["pitch"]),
                                        float(pose_dict["yaw"]),
                                        "sxyz",
                                    )
                                    qx = str(quat[1])
                                    qy = str(quat[2])
                                    qz = str(quat[3])
                                    qw = str(quat[0])

                                    lfd_execution = subprocess.Popen(
                                        [
                                            "ros2",
                                            "launch",
                                            "lfd_executor",
                                            "lfd.launch.py",
                                            "publish_rate:=" + control_rate,
                                            "controller_type:=" + controller_type,
                                            "repetition:=" + pose_dict["description"],
                                            "data_folder:=" + trajectories_path,
                                        ],
                                        stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE,
                                    )

                                    os.set_blocking(lfd_execution.stdout.fileno(), False)

                                    print("LFD is running")

                                    time.sleep(5)

                                    sim_test = subprocess.Popen(
                                        [
                                            "ros2",
                                            "run",
                                            "lfd_executor",
                                            "simulation_testing",
                                            "--ros-args",
                                            "-p",
                                            "noise_type:=" + noise_type,
                                            "-p",
                                            "noise_dimension:=" + noise_dim,
                                            "-p",
                                            "noise_magnitude:=" + noise_magnitude,
                                            "-p",
                                            "noise_time:=" + noise_time,
                                            "-p",
                                            "reg_factor:=" + reg_factor,
                                            "-p",
                                            "x:=" + x,
                                            "-p",
                                            "y:=" + y,
                                            "-p",
                                            "z:=" + z,
                                            "-p",
                                            "qx:=" + qx,
                                            "-p",
                                            "qy:=" + qy,
                                            "-p",
                                            "qz:=" + qz,
                                            "-p",
                                            "qw:=" + qw,
                                            "-p",
                                            "publish_rate:=" + logging_rate,
                                            "-p",
                                            "controller_type:=" + controller_type,
                                            "-p",
                                            "repetition:=" + pose_dict["description"],
                                            "-p",
                                            "data_folder:=" + data_folder,
                                        ],
                                        stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE,
                                    )

                                    warnings = 0

                                    end_time = time.time() + 30.0

                                    while not sim_test.poll() and time.time() < end_time:
                                        # Print stdout and stderr of lfd execution
                                        output_line = lfd_execution.stdout.readline().decode("utf-8")
                                        if output_line != "":
                                            if "[WARN]" in output_line:
                                                print("Warning received, repeating run")
                                                # Kill processes if warning is detected
                                                sim_test.send_signal(signal.SIGINT)
                                                # Print stdout of lfd execution
                                                print("Warning: " + output_line)
                                                # Lower counter to repeat test, make sure it only happens once, even if more warnings are received
                                                if warnings < 1:
                                                    i -= 1
                                                warnings += 1

                                        # Sleep if no output is available
                                        time.sleep(0.1)

                                    # Check if lfd_execution got stuck
                                    while not sim_test.poll():
                                        print("LFD execution got stuck, repeating run")
                                        # Kill processes if warning is detected
                                        sim_test.send_signal(signal.SIGINT)
                                        lfd_execution.send_signal(signal.SIGTERM)
                                        # Lower counter to repeat test
                                        if warnings < 1:
                                            i -= 1
                                        warnings += 1
                                        # Escalate if stuck
                                        if warnings > 10:
                                            while "/simulation_testing" in subprocess.run(
                                                ["ros2", "node", "list"], stdout=subprocess.PIPE, stderr=subprocess.PIPE
                                            ).stdout.decode("utf-8").split("\n"):
                                                print("Aborting LFD execution")
                                                subprocess.run(["killall", "-9", "simulation_testing"])
                                                time.sleep(2.0)
                                            break
                                        time.sleep(1.0)

                                    print("LFD is terminating")
                                    lfd_execution.send_signal(signal.SIGTERM)
                                    sim_test.send_signal(signal.SIGTERM)

                                    # Shut down relaxed frame broadcaster and tcp for ik broadcaster
                                    relaxed_frame_broadcaster.send_signal(signal.SIGTERM)
                                    tcp_for_ik_broadcaster.send_signal(signal.SIGTERM)

                                    tcp_for_ik_broadcaster.wait(timeout=10)

                                    while lfd_execution.poll() is None:
                                        print("Waiting: LFD is terminating")
                                        time.sleep(1)

                                    while sim_test.poll() is None:
                                        print("Waiting: Simulation testing is terminating")
                                        time.sleep(1)

                                    # Shut down all but durable nodes and ur5e nodes
                                    exception_nodes = durable_nodes + ur5e_nodes
                                    kill_ros_nodes(exceptions=exception_nodes)

                                    # If warnings were received
                                    if warnings > 0:
                                        print("Warnings received, checking if file was stored")
                                        # Check if this result string is already in the list of performed tests
                                        current_trajectory_files = next(os.walk(trajectories_path))[2]
                                        peformed_check = sum([True for file in current_trajectory_files if file.startswith(result_string)])
                                        if peformed_check > 0:
                                            # Get string of actual file
                                            actual_file = [file for file in current_trajectory_files if file.startswith(result_string)][0]
                                            # Delete file
                                            file_with_path = os.path.join(trajectories_path, actual_file)
                                            os.remove(file_with_path)
                                            print("Deleted file: " + actual_file)

                                        sim_test.send_signal(signal.SIGKILL)

                                    # Raise counter
                                    i += 1

                                # Shut down fake UR5e
                                fake_ur5e.send_signal(signal.SIGINT)

                                # Wait for fake UR5e to terminate
                                fake_ur5e.wait(timeout=10)

                                # Shut down all but durable nodes
                                kill_ros_nodes(exceptions=durable_nodes)


except KeyboardInterrupt:
    print("Keyboard interrupt, starting clean shutdown")
    pass


# Shut down all nodes
kill_ros_nodes()
