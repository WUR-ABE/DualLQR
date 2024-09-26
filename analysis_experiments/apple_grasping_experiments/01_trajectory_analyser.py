import os
import numpy as np
import pickle
import transforms3d as t3d
import matplotlib.pyplot as plt

# Get list of data
data_path = "!!SET_LOCATION_OF_DATA_FOLDER!!"

trajectories_path = os.path.join(data_path, "testing_suction_trajectories")
filelist = os.listdir(trajectories_path)

# Load results
data = {}
models = []
for i, file in enumerate(filelist):
    try:
        if file.endswith(".pickle") and file.startswith("result"):
            with open(os.path.join(trajectories_path, file), "rb") as f:
                data[file[:-7]] = pickle.load(f)
        if file.endswith(".pickle") and file.startswith("LQR"):
            models += [file[:-7]]
        print("\r Loaded file {} of {}".format(i + 1, len(filelist)), end="")
    except:
        print("\r Failed to load file {}".format(file), end="")

model_name_floats = []

for model_name in models:
    model_name_floats += [float(model_name.split("-")[-1])]

# Function to analyse a run
def analyse_run(key, close_threshold=0.05, x_threshold=0.03, z_threshold=0.1, roll_threshold=0.07, pitch_threshold=0.07, yaw_threshold=0.07):
    # Get properties of run out of key
    properties = key.split("_")

    # Get value of each property and put in result of this run
    result = []
    for prop in properties:
        parts = prop.split("-")
        if len(parts) == 1:
            continue
        elif len(parts) == 2:
            result += [parts[1]]
        elif len(parts) == 3:
            result += [parts[1] + "-" + parts[2]]
    
    run_data = data[key]
    transform_data = run_data[1]

    # Extract controls
    controls = run_data[0]
    time_start = controls[0, 0]
    time_end = controls[-1, 0]
    time_diff = time_end - time_start

    # Determine moment vacuum pressure increased
    vacuum_controls = controls[:, 37]
    # Get moment vacuum controls went above 1.0 and substract time_start
    try:
        time_to_vacuum = controls[np.where(vacuum_controls > 1.0)[0][0], 0] - time_start
        time_of_vacuum = controls[np.where(vacuum_controls > 1.0)[0][0], 0]
    except IndexError:
        time_to_vacuum = time_end - time_start
        time_of_vacuum = time_end

    # Extract goal and robot pose data
    goal_pose_data = []
    robot_pose_data = []
    robot_pose_in_goal_frame_data = []
    for data_row in transform_data[np.where(transform_data[:, 0] < time_of_vacuum)]:
        # Get goal pose and append
        roll, pitch, yaw = t3d.euler.quat2euler((data_row[7], data_row[4], data_row[5], data_row[6]), axes="sxyz")
        goal_pose_data += [np.hstack([data_row[0:8], (roll, pitch, yaw)])]
        # Get robot pose and append
        roll, pitch, yaw = t3d.euler.quat2euler((data_row[15], data_row[12], data_row[13], data_row[14]), axes="sxyz")
        robot_pose_data += [np.hstack([data_row[8:16], (roll, pitch, yaw)])]

        # Calculate goal pose transformation matrix
        relaxed_to_goal_tf = np.eye(4)
        relaxed_to_goal_tf[0:3, 0:3] = t3d.quaternions.quat2mat((data_row[7], data_row[4], data_row[5], data_row[6]))
        relaxed_to_goal_tf[0:3, 3] = data_row[1:4]

        # Calculate robot pose transformation matrix
        relaxed_to_robot_tf = np.eye(4)
        relaxed_to_robot_tf[0:3, 0:3] = t3d.quaternions.quat2mat((data_row[15], data_row[12], data_row[13], data_row[14]))
        relaxed_to_robot_tf[0:3, 3] = data_row[9:12]

        # Calculate robot pose in goal frame
        robot_pose_in_goal_frame = np.dot(np.linalg.inv(relaxed_to_goal_tf), relaxed_to_robot_tf)

        # Get roll, pitch, yaw from robot pose in goal frame
        roll, pitch, yaw = t3d.euler.mat2euler(robot_pose_in_goal_frame[0:3, 0:3], axes="sxyz")
        q0, q1, q2, q3 = t3d.quaternions.mat2quat(robot_pose_in_goal_frame[0:3, 0:3])
        robot_pose_in_goal_frame_data += [np.hstack([data_row[0], robot_pose_in_goal_frame[0:3, 3], (q1, q2, q3, q0), (roll, pitch, yaw)])]


    goal_pose_data = np.asarray(goal_pose_data)
    robot_pose_data = np.asarray(robot_pose_data)
    goal_robot_diff = np.asarray(robot_pose_in_goal_frame_data)

    # Get row from where x difference is continuously smaller than close_threshold
    close_array = np.where(np.abs(goal_robot_diff[:, 2]) < close_threshold)[0]
    try:
        large_differences = [close_array[0]]
        for i in range(len(close_array) - 1):
            if close_array[i + 1] - close_array[i] > 1:
                large_differences += [close_array[i + 1]]
        last_diff_row = large_differences[-1]

        # Get last set of rows where x difference is continuously smaller than close_threshold
        goal_robot_diff = goal_robot_diff[last_diff_row:, :]
    except IndexError:
        goal_robot_diff = goal_robot_diff[np.where(np.abs(goal_robot_diff[:, 2]) < close_threshold)]

    # Caluculate euclidean distance at those rows
    goal_robot_euclid = np.linalg.norm(goal_robot_diff[:, 1:4], axis=1)

    # Calculate rows outside of Y, Z, roll, pitch, yaw threshold
    goal_robot_error = goal_robot_diff[
        (np.abs(goal_robot_diff[:, 1]) > x_threshold)
        | (np.abs(goal_robot_diff[:, 3]) > z_threshold)
        | (np.abs(goal_robot_diff[:, 8]) > roll_threshold)
        | (np.abs(goal_robot_diff[:, 9]) > pitch_threshold)
        | (np.abs(goal_robot_diff[:, 10]) > yaw_threshold)
    ]

    number_close_rows = len(goal_robot_diff)
    number_error_rows = len(goal_robot_error)
    try:
        fract_error = float(len(goal_robot_error)) / float(len(goal_robot_diff))
    except ZeroDivisionError:
        fract_error = 1.0
    close_euclid = np.mean(goal_robot_euclid)

    precision = 1 - fract_error

    # If frequency was higher than 30 Hz, this should be taken into account
    if float(result[2]) > 30:
        dist_robot_pose_data = robot_pose_data[::int(float(result[2]) / 30)]
    else:
        dist_robot_pose_data = robot_pose_data

    # Determine euclidean distance and rotated distance
    total_euclid_dist = 0
    total_rotated_dist = 0
    for i in range(len(dist_robot_pose_data) - 1):
        # Calculate euclidean distance
        pos_diff = dist_robot_pose_data[i + 1, :4] - dist_robot_pose_data[i, :4]
        total_euclid_dist += np.linalg.norm(pos_diff[1:])

        # Calculate rotated distance
        qw, qx, qy, qz = dist_robot_pose_data[i, 7], dist_robot_pose_data[i, 4], dist_robot_pose_data[i, 5], dist_robot_pose_data[i, 6]
        old_q = pq.Quaternion(w=qw, x=qx, y=qy, z=qz)

        qw, qx, qy, qz = dist_robot_pose_data[i + 1, 7], dist_robot_pose_data[i + 1, 4], dist_robot_pose_data[i + 1, 5], dist_robot_pose_data[i + 1, 6]
        new_q = pq.Quaternion(w=qw, x=qx, y=qy, z=qz)

        rotated_dist = pq.Quaternion.absolute_distance(old_q, new_q)
        total_rotated_dist += rotated_dist

    # Determine position error at final pose
    try:
        final_position_error = np.linalg.norm(goal_robot_diff[-1, 1:4])
    except IndexError:
        final_position_error = 10.0

    # Determine orientation error at final pose
    try:
        final_robot_quat = pq.Quaternion(w=robot_pose_data[-1, 7], x=robot_pose_data[-1, 4], y=robot_pose_data[-1, 5], z=robot_pose_data[-1, 6])
        final_goal_quat = pq.Quaternion(w=goal_pose_data[-1, 7], x=goal_pose_data[-1, 4], y=goal_pose_data[-1, 5], z=goal_pose_data[-1, 6])
        final_orientation_error = pq.Quaternion.absolute_distance(final_robot_quat, final_goal_quat)
    except IndexError:
        final_orientation_error = 10.0
    
    result += [len(goal_pose_data)]
    result += [total_euclid_dist]
    result += [total_rotated_dist]
    result += [final_position_error]
    result += [final_orientation_error]
    result += [time_to_vacuum]
    result += [number_close_rows]
    result += [number_error_rows]
    result += [fract_error]
    result += [close_euclid]
    result += [precision]

    return result


analysed_data = [
    [
        "reg_factor",
        "controller_type",
        "controller_rate",
        "lookahead_time",
        "gain",
        "time_stamp",
        "total_rows",
        "total_euclid",
        "total_rotated",
        "final_position_error",
        "final_orientation_error",
        "time_to_vacuum",
        "end_number_close_rows", 
        "end_number_error_rows", 
        "end_fract_error", 
        "end_close_euclid", 
        "end_precision",
    ]
]

for i, key in enumerate(data.keys()):
    result = analyse_run(key)

    # Put results of this run into analysed data
    analysed_data += [result]
    print("\r Analysed run {} of {}".format(i + 1, len(data.keys())), end="")

# Save analysed data to csv
data_array = np.asarray(analysed_data)
store_path = os.path.join(data_path, "analysis_files", "analysed_data_suction.csv")
np.savetxt(store_path, data_array, delimiter=";", fmt="%s")
