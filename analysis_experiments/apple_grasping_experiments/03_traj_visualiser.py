import os
import numpy as np
import pickle
import transforms3d as t3d
import matplotlib.pyplot as plt
import pandas as pd
import matplotlib as mpl

# Set the fonttype to TrueType
mpl.rcParams['text.usetex'] = True

# Font sizes
SMALL_SIZE = 11
MEDIUM_SIZE = 12
BIGGER_SIZE = 16

plt.rc('font', size=SMALL_SIZE)          # controls default text sizes
plt.rc('axes', titlesize=BIGGER_SIZE)     # fontsize of the axes title
plt.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('ytick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('legend', fontsize=SMALL_SIZE)    # legend fontsize
plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title

# Load data from CSV
data_path = "!!SET_LOCATION_OF_DATA_FOLDER!!"

trajectories_path = os.path.join(data_path, "testing_suction_trajectories")
filelist = os.listdir(trajectories_path)

# Load results
data = []
models = []
for i, file in enumerate(filelist):
    if file.endswith(".pickle") and file.startswith("result"):
        data += [file[:-7]]
    elif file.endswith(".pickle") and file.startswith("LQR"):
        models += [file[:-7]]

model_name_floats = []

for model_name in models:
    model_name_floats += [float(model_name.split("-")[-1])]

# column names for keys
keys = [
    [
        "run",
        "reg_factor",
        "controller_type",
        "controller_rate",
        "lookahead_time",
        "gain",
        "time_stamp",
    ]
]

# Split data into runs and load into DF
for key in data:
    # Get properties of run out of key
    properties = key.split("_")

    # Get value of each property and put in result of this run
    result = [key]
    for prop in properties:
        parts = prop.split("-")
        if len(parts) == 1:
            continue
        elif len(parts) == 2:
            result += [parts[1]]
        elif len(parts) == 3:
            result += [parts[1] + "-" + parts[2]]

    if len(result) < 7:
        diff = 10 - len(result)
        for i in range(diff):
            result.insert(-1, "none")

    keys += [result]

# Create dataframe
keys_df = pd.DataFrame(keys[1:], columns=keys[0])

# Select the three runs with matching video
best_models = [
    "1715863862.6071444",
    "1715853065.0440514",
    "1715856696.1505072",
]

# Select rows with best model for each controller type
filtered_df = keys_df[keys_df.time_stamp.isin(best_models)].reset_index(drop=True)

# Define headers
headers = ["X (m)", "Y (m)", "Z", "roll", "pitch", "yaw"]

# Loop over runs
for i, df_row in filtered_df.iterrows():
    data_name = df_row.run

    # Load data
    with open(os.path.join(trajectories_path, data_name + ".pickle"), "rb") as f:
        traj_data = pickle.load(f)

    transform_data = traj_data[1]

    controls = traj_data[0]

    try:
        # Determine moment vacuum pressure increased
        vacuum_controls = controls[:, 37]
        # Get moment vacuum controls went above 1.0 and substract time_start
        time_of_vacuum = controls[np.where(vacuum_controls > 1.0)[0][0], 0]
    except IndexError:
        time_of_vacuum = controls[-1,0]

    # Extract goal pose data
    goal_pose_data = []
    for row in transform_data:
        roll, pitch, yaw = t3d.euler.quat2euler((row[7], row[4], row[5], row[6]), axes="sxyz")
        goal_pose_data += [np.hstack([row[0:4], (roll, pitch, yaw)])]

    goal_pose_data = np.asarray(goal_pose_data)

    # Filter to time before end of trajectory
    goal_pose_data = goal_pose_data[(goal_pose_data[:, 0] <= time_of_vacuum + 0.1)]

    # Extract robot pose data
    robot_pose_data = []
    for row in transform_data:
        roll, pitch, yaw = t3d.euler.quat2euler((row[15], row[12], row[13], row[14]), axes="sxyz")
        robot_pose_data += [np.hstack([row[8:12], (roll, pitch, yaw)])]
    robot_pose_data = np.asarray(robot_pose_data)

    # Filter to time before end of trajectory
    robot_pose_data = robot_pose_data[(robot_pose_data[:, 0] <= time_of_vacuum + 0.1)]
    
    # Get difference between goal and robot pose
    goal_robot_diff = goal_pose_data[:, :2] - robot_pose_data[:, :2]

    # Reset time column
    goal_robot_diff[:, 0] = goal_pose_data[:, 0]

    # Get rows where x difference is smaller than 0.05
    goal_robot_diff = goal_robot_diff[np.where(np.abs(goal_robot_diff[:, 1]) < 0.05)]

    # Get rows where time difference with next row is bigger than 0.01 seconds
    time_difference = goal_robot_diff[1:, 0] - goal_robot_diff[:-1, 0]
    large_time_diffs = np.where(time_difference > 0.01)[0]
    if len(large_time_diffs) != 0:
        final_time_diff = large_time_diffs[-1]
        goal_robot_diff = goal_robot_diff[final_time_diff:]

    # Determine first time of x difference smaller than 0.05
    time_close = goal_robot_diff[0, 0]

    # Filter robot and goal pose data to after time_close
    robot_pose_data = robot_pose_data[(robot_pose_data[:, 0] >= time_close)]
    goal_pose_data = goal_pose_data[(goal_pose_data[:, 0] >= time_close)]

    time_start = traj_data[0][0, 0]

    # plotting
    fig, ax = plt.subplots(nrows=2, ncols=1)
    fig.set_size_inches(2, 5/1.5)

    # position plotting
    for j in range(rows):
        plot_row = j

        # Add headers
        ax[j].set_ylabel(headers[plot_row])

        # Add path of manipulator
        ax[j].plot((robot_pose_data[:, 0] - time_start), robot_pose_data[:, plot_row + 1], "b", lw=3)

        # Add path of goal
        ax[j].plot((goal_pose_data[:, 0] - time_start), goal_pose_data[:, plot_row + 1], "r", lw=1.5)

        # Add vertical line at moment of vacuum increase
        ax[j].axvline(x=(time_of_vacuum - time_start), color="green", linestyle="--")

        # Format labels
        ax[j].yaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: "{:.1f}".format(x)))

    # Add x axis label
    ax[1].set_xlabel("Time (s)")

    # Store plot
    fig.align_ylabels()
    plt.tight_layout()
    plot_name = "suction_example_" + df_row.reg_factor + ".pdf"
    store_path = os.path.join(data_path, "analysis_files", "plots", plot_name)
    plt.savefig(store_path, format="pdf", bbox_inches="tight")
