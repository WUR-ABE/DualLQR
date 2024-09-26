import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import numpy as np
import os
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

df_path = os.path.join(data_path, "analysis_files", "analysed_data_suction.csv")
df = pd.read_csv(df_path, delimiter=";")


# Function to invert fract_error values so that higher values are better, still going from 0 to 1
df["combined_distance"] = df["total_euclid"] + df["total_rotated"]

#%% Create boxplots for each control cost
fig, axs = plt.subplots(nrows=3, figsize=(6, 5.))
# fig.suptitle("Line Plot of Fract Error, Euclidean Distance, and rotated distance (Controller Type: {})".format(controller))

# Create line plot of Fract Error with error bars
sns.boxplot(
    x="reg_factor",
    y="end_precision",
    data=df,
    ax=axs[0],
    fill=False,
    color=sns.cubehelix_palette(4)[-1],
)

# Add horizontal line at y=0.88, behind the plot
axs[0].axhline(y=0.88, color="black", linestyle="--", zorder=0)


# Set title with dimensions of noise
# axs[0, index].set_title("Dimensions {}, {}, and {}".format(dimension[0], dimension[1], dimension[2]))
axs[0].set_ylabel("Accuracy")
axs[0].set_xlabel("Control cost")
# Set decimal places for y-axis
axs[0].yaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: "{:.1f}".format(x)))

# Create line plot of Euclidean Distance with error bars
sns.boxplot(
    x="reg_factor",
    y="combined_distance",
    data=df,
    ax=axs[1],
    fill=False,
    color=sns.cubehelix_palette(4)[-1],
)
axs[1].set_ylabel("Distance")
axs[1].set_xlabel("Control cost")
# Set decimal places for y-axis
axs[1].yaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: "{:.1f}".format(x)))

# Create line plot of Rotated Distance with error bars
sns.boxplot(
    x="reg_factor",
    y="time_to_vacuum",
    data=df,
    ax=axs[2],
    fill=False,
    color=sns.cubehelix_palette(4)[-1],
)
axs[2].set_xlabel("Control cost")
axs[2].set_ylabel("Time (s)")
# Set decimal places for y-axis
axs[2].yaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: "{:.1f}".format(x)))

# Unsuccesful attempts
attempt = df[df["time_to_vacuum"] == df["time_to_vacuum"].max()]

# Place a white square at final_precision, at column -0.3
axs[0].plot(9, attempt["end_precision"].item(), "ws", markersize=10)

# Place a cross at final_precision, at column -0.3
axs[0].plot(9, attempt["end_precision"].item(), "kx", markersize=6)

# Place a white square at combined distance, at column -0.3
axs[1].plot(9, attempt["combined_distance"].item(), "ws", markersize=10)

# Place a cross at combined distance, at column -0.3
axs[1].plot(9, attempt["combined_distance"].item(), "kx", markersize=6)

# Place a white square at max time, at column -0.3
axs[2].plot(9, attempt["time_to_vacuum"].item(), "ws", markersize=10)

# Place a cross at max time, at column -0.3
axs[2].plot(9, attempt["time_to_vacuum"].item(), "kx", markersize=6)

# Align y-axis labels
fig.align_ylabels(axs)

# Adjust layout and display the plot
plt.tight_layout()
filename = "DualLQR_boxplot_suction.pdf"
store_path = os.path.join(data_path, "analysis_files", "plots", filename)
plt.savefig(store_path, format="pdf", bbox_inches="tight")
plt.show()
