import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import numpy as np
import os
import matplotlib as mpl

# Set the fonttype to TrueType
mpl.rcParams["text.usetex"] = True

# Font sizes
SMALL_SIZE = 11
MEDIUM_SIZE = 13
BIGGER_SIZE = 14

plt.rc('font', size=SMALL_SIZE)          # controls default text sizes
plt.rc('axes', titlesize=BIGGER_SIZE)     # fontsize of the axes title
plt.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=MEDIUM_SIZE)    # fontsize of the tick labels
plt.rc('ytick', labelsize=MEDIUM_SIZE)    # fontsize of the tick labels
plt.rc('legend', fontsize=SMALL_SIZE)    # legend fontsize
plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title

# Load data from CSV
data_path = "!!SET_LOCATION_OF_DATA_FOLDER!!"

# Load combined data
df_path = os.path.join(data_path, "analysis_files", "analysed_data.csv")
df = pd.read_csv(df_path, delimiter=";")

# Filter the DataFrame for the "no noise" runs
df_no_noise = df[df["noise_type"] == "none"]

# Duplicate the "no noise" rows and modify them for step and sinus cases
for dim in ["x", "y", "z", "roll", "pitch", "yaw"]:
    df_no_noise_sinus = df_no_noise.copy()
    df_no_noise_sinus["noise_dimension"] = dim
    df_no_noise_sinus["noise_type"] = "sinus"
    df_no_noise_sinus["noise_rate"] = 1.0

    # Concatenate the modified "no noise" rows with the original DataFrame
    df = pd.concat([df, df_no_noise_sinus], ignore_index=True)

# Create a mapping for noise magnitude and noise dimension to noise level
noise_level_map = {
    (0.0, "x"): 0,
    (0.0, "y"): 0,
    (0.0, "z"): 0,
    (0.0, "roll"): 0,
    (0.0, "pitch"): 0,
    (0.0, "yaw"): 0,
    (0.05, "x"): 1,
    (0.05, "y"): 1,
    (0.05, "z"): 1,
    (0.10, "x"): 2,
    (0.10, "y"): 2,
    (0.10, "z"): 2,
    (0.15, "x"): 3,
    (0.15, "y"): 3,
    (0.15, "z"): 3,
    (0.15, "roll"): 1,
    (0.15, "pitch"): 1,
    (0.15, "yaw"): 1,
    (0.30, "roll"): 2,
    (0.30, "pitch"): 2,
    (0.30, "yaw"): 2,
    (0.45, "roll"): 3,
    (0.45, "pitch"): 3,
    (0.45, "yaw"): 3,
}

# Function to map noise_magnitude and noise_dimension to noise level
def map_to_noise_level(row):
    return noise_level_map.get((row["noise_magnitude"], row["noise_dimension"]), np.nan)

# Apply mapping function to create 'noise_level' column
df["noise_level"] = df.apply(map_to_noise_level, axis=1)

df["combined_distance"] = df["total_euclid"] + df["total_rotated"]

# Create a mapping for noise level and noise dimension to pos ori
pos_ori_map = {
    (0, "x"): "none",
    (0, "y"): "none",
    (0, "z"): "none",
    (0, "roll"): "none",
    (0, "pitch"): "none",
    (0, "yaw"): "none",
    (1, "x"): "position",
    (1, "y"): "position",
    (1, "z"): "position",
    (1, "roll"): "orientation",
    (1, "pitch"): "orientation",
    (1, "yaw"): "orientation",
    (2, "x"): "position",
    (2, "y"): "position",
    (2, "z"): "position",
    (2, "roll"): "orientation",
    (2, "pitch"): "orientation",
    (2, "yaw"): "orientation",
    (3, "x"): "position",
    (3, "y"): "position",
    (3, "z"): "position",
    (3, "roll"): "orientation",
    (3, "pitch"): "orientation",
    (3, "yaw"): "orientation",
}

# Function to map noise dimension to pos ori
def map_to_pos_ori(row):
    return pos_ori_map.get((row["noise_level"], row["noise_dimension"]), np.nan)

# Apply mapping function to create 'pos_ori' column
df["pos_ori"] = df.apply(map_to_pos_ori, axis=1)

# %% Table with best model of each controller type

# Best model is based on reg_factor, set reg_factor for each controller type
best_models = [
    ["InfLQR", 0.0],
    ["DualLQR", 0.0],
]

# Select rows with best model for each controller type
best_models_df = df[
    (
        ((df["controller_type"] == best_models[0][0]) & (df["reg_factor"] == best_models[0][1]))
        | ((df["controller_type"] == best_models[1][0]) & (df["reg_factor"] == best_models[1][1]))
    )
    & (df["noise_type"] == "sinus")
]

# Select columns with data
best_models_df = best_models_df[
    [
        "controller_type",
        "noise_level",
        "total_euclid",
        "total_rotated",
        "combined_distance",	
        "pos_ori",
        "end_precision",
    ]
]

# Combine the rows from each model into one row, calculate the mean and standard deviation
best_models_grouped_df = best_models_df.groupby(["noise_level", "pos_ori", "controller_type", ]).agg(["mean", "std"])

# Relevant columns
relevant_columns = [
    "end_precision",
    "total_euclid",
    "total_rotated",
]

# Create a LaTeX table with only the relevant columns
print(best_models_grouped_df[relevant_columns].to_latex(float_format="%.4f", label="tab:best_models", caption="Best models for each controller type."))

from scipy.stats import ttest_ind
# Find significant differences between the best models
# Create a dictionary to store the p-values
t_stats = {}
p_values = {}

# Loop through each relevant column
for noise_level in [0, 1, 2, 3]:
    # Get the data for the current column
    data = best_models_df[best_models_df["noise_level"] == noise_level]

    # Create a dictionary to store the p-values for the current column
    p_values[noise_level] = {}
    t_stats[noise_level] = {}

    # Loop through noise levels and pos_ori
    for pos_ori in data["pos_ori"].unique():
        for column in relevant_columns:
            # Filter the data for the current noise level and pos_ori
            filtered_data = data[(data["pos_ori"] == pos_ori)]

            # Get the data for the two controllers
            controller1_data = filtered_data[filtered_data["controller_type"] == best_models[0][0]][column]
            controller2_data = filtered_data[filtered_data["controller_type"] == best_models[1][0]][column]

            # Perform a t-test
            t_stat, p_value = ttest_ind(controller1_data, controller2_data)

            # Store the p-value in the dictionary
            t_stats[noise_level][(pos_ori, column)] = t_stat
            p_values[noise_level][(pos_ori, column)] = p_value

#%% Lineplots

max_distance = df[((df["controller_type"] == "InfLQR")| (df["controller_type"] == "DualLQR"))]["combined_distance"].max()
min_distance = df[((df["controller_type"] == "InfLQR")| (df["controller_type"] == "DualLQR"))]["combined_distance"].min()

for controller in ["InfLQR", "DualLQR"]: 
    # Create subplots for each combination of controller and dimension
    fig, axs = plt.subplots(nrows=2, ncols=2, figsize=(6, 5/1.5))

    semi_filtered_df = df[(df["controller_type"] == controller)]

    for index, dimension in enumerate([["x", "y", "z"], ["roll", "pitch", "yaw"]]):
        # Filter the DataFrame for the current controller and noise dimension
        filtered_df = semi_filtered_df[
            (
                (semi_filtered_df["noise_dimension"] == dimension[0])
                | (semi_filtered_df["noise_dimension"] == dimension[1])
                | (semi_filtered_df["noise_dimension"] == dimension[2])
            )
        ]

        # Create line plot of Fract Error with error bars
        sns.lineplot(
            x="reg_factor",
            y="end_precision",
            hue="noise_level",
            data=filtered_df,
            markers=True,
            err_style="bars",
            ax=axs[0, index],
            palette=sns.cubehelix_palette(4),
        )

        # Add horizontal line at y=0.88, behind the plot
        axs[0, index].axhline(y=0.88, color="black", linestyle="--", zorder=0)

        # Remove legend
        axs[0, index].get_legend().remove()
        # Remove x-axis ticks
        axs[0, index].set_xticks(np.arange(-3.0, 3.1, 0.6))
        axs[0, index].tick_params(labelbottom=False)
        # Remove x-axis label
        axs[0, index].set_xlabel("")

        # Set ax limits from 0 to 1
        axs[0, index].set_ylim(-0.05, 1.05)
        axs[0, index].set_yticks(np.arange(0.0, 1.1, 0.5))
        # Set y ticks to have one decimal place
        axs[0, index].yaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: "{:.1f}".format(x)))

        # Set subfigure title
        if dimension == ["x", "y", "z"]:
            axs[0, index].set_title("Position oscillation")
            axs[0, index].set_ylabel("Accuracy")
        elif dimension == ["roll", "pitch", "yaw"]:
            axs[0, index].set_title("Orientation oscillation")
            axs[0, index].tick_params(labelleft=False)
            axs[0, index].set_ylabel("")

        # Create line plot of Euclidean Distance with error bars
        sns.lineplot(
            x="reg_factor",
            y="combined_distance",
            hue="noise_level",
            data=filtered_df,
            markers=True,
            err_style="bars",
            ax=axs[1, index],
            palette=sns.cubehelix_palette(4),
        )
        
        # Remove legend
        axs[1, index].get_legend().remove()
        axs[1, index].tick_params(axis="x", rotation=90)
        axs[1, index].set_xticks(np.arange(-3.0, 3.1, 0.6))

        axs[1, index].set_ylim(min_distance, max_distance)
        # Set y ticks to have one decimal place
        axs[1, index].yaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: "{:.1f}".format(x)))

        if dimension == ["x", "y", "z"]:
            axs[1, index].set_ylabel("Distance")
        elif dimension == ["roll", "pitch", "yaw"]:	
            axs[1, index].tick_params(labelleft=False)
            axs[1, index].set_ylabel("")
        axs[1, index].set_xlabel("Control cost")

    # Add legend to the first subplot
    axs[0, 0].legend(("None", "Low", "Medium", "High"), loc="lower left", ncol=2)

    # Adjust layout and display the plot
    fig.align_ylabels()
    plt.tight_layout()
    filename = controller + "_lineplot.pdf"
    store_path = os.path.join(data_path, "analysis_files", "plots", filename)
    plt.savefig(store_path, format="pdf", bbox_inches="tight")
    plt.show()
