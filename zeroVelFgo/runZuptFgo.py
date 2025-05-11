import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import cumfreq
import gtsam
from zeroVelFgoEstimator import ZeroVelFgoEstimator, MeasType, GtsamOptimizerType
import concurrent.futures
import plotly.express as px
import plotly.graph_objects as go
from plotly.subplots import make_subplots


def plot_measurement_errors(position_meas, position_gt, scatter_size=10):
    # Define time scale
    dt = 1 / 60
    position_meas["time"] = position_meas["time_index"] * dt

    # Interpolate ground truth data to match measurement timestamps
    position_gt_interpolated = (
        position_gt.set_index("time_index")
        .reindex(position_meas["time_index"])
        .interpolate()
        .reset_index()
    )

    # Compute error for right foot
    error_right = (
        (position_meas["pos_right_x"] - position_gt_interpolated["pos_right_x"]) ** 2
        + (position_meas["pos_right_y"] - position_gt_interpolated["pos_right_y"]) ** 2
    ) ** 0.5

    # Compute error for left foot
    error_left = (
        (position_meas["pos_left_x"] - position_gt_interpolated["pos_left_x"]) ** 2
        + (position_meas["pos_left_y"] - position_gt_interpolated["pos_left_y"]) ** 2
    ) ** 0.5

    # Create a 2x1 subplot layout
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))

    # Left foot error scatter plot
    axes[0].scatter(position_meas["time"], error_left, s=scatter_size, color="blue")
    axes[0].set_title("Left Foot Pos. Meas. VS GT")
    axes[0].set_xlabel("Time (s)")
    axes[0].set_ylabel("Error (m)")
    axes[0].grid()
    axes[0].set_ylim(0, 5)  # Set y-axis limit

    # Right foot error scatter plot
    axes[1].scatter(position_meas["time"], error_right, s=scatter_size, color="red")
    axes[1].set_title("Right Foot Pos. Meas. VS GT")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Error (m)")
    axes[1].grid()
    axes[1].set_ylim(0, 5)  # Set y-axis limit

    plt.tight_layout()
    plt.savefig("figure/position_meas_vs_gt.eps")


def obtain_gtsam_restuls(
    imu_right_file,
    imu_left_file,
    position_meas_file,
    read_zupt_from_csv: False,
    read_zupt_step_from_csv: False,
    read_zupt_pos_from_csv: False,
    read_zupt_pos_step_from_csv: False,
) -> tuple:

    isam_0vel_pd = pd.DataFrame()
    isam_0vel_step_pd = pd.DataFrame()
    isam_0vel_pos_pd = pd.DataFrame()
    isam_0vel_pos_step_pd = pd.DataFrame()

    if read_zupt_from_csv:
        isam_0vel_pd = pd.read_csv("results/results_fgo_zupt.csv")

    if read_zupt_step_from_csv:
        isam_0vel_step_pd = pd.read_csv("results/results_fgo_zupt_step.csv")

    if read_zupt_pos_from_csv:
        isam_0vel_pos_pd = pd.read_csv("results/results_fgo_zupt_pos.csv")

    if read_zupt_pos_step_from_csv:
        isam_0vel_pos_step_pd = pd.read_csv("results/results_fgo_zupt_pos_step.csv")

    def run_estimator(use_pos_meas, use_max_foot, use_zero_vel):
        """Helper function to run the estimator."""
        estimator = ZeroVelFgoEstimator(
            imu_right_file,
            imu_left_file,
            position_meas_file,
            use_pos_meas=use_pos_meas,
            use_max_foot=use_max_foot,
            use_zero_vel=use_zero_vel,
            gtsam_optimizer_type=GtsamOptimizerType.ISAM2,
        )
        estimator.run_zero_vel_fgo()
        return estimator

    with concurrent.futures.ThreadPoolExecutor() as executor:
        # Run the estimator with different configurations
        if not read_zupt_from_csv:
            isam_0vel = executor.submit(
                run_estimator, use_pos_meas=False, use_max_foot=False, use_zero_vel=True
            )

        if not read_zupt_step_from_csv:
            isam_0vel_step = executor.submit(
                run_estimator, use_pos_meas=False, use_max_foot=True, use_zero_vel=True
            )

        if not read_zupt_pos_from_csv:
            isam_0vel_pos = executor.submit(
                run_estimator, use_pos_meas=True, use_max_foot=False, use_zero_vel=True
            )

        if not read_zupt_pos_step_from_csv:
            isam_0vel_pos_step = executor.submit(
                run_estimator, use_pos_meas=True, use_max_foot=True, use_zero_vel=True
            )

    if not read_zupt_from_csv:
        isam_0vel_pd = isam_0vel.result().result_pd
        isam_0vel_pd.to_csv("results/results_fgo_zupt.csv", index=False)
    if not read_zupt_step_from_csv:
        isam_0vel_step_pd = isam_0vel_step.result().result_pd
        isam_0vel_step_pd.to_csv("results/results_fgo_zupt_step.csv", index=False)
    if not read_zupt_pos_from_csv:
        isam_0vel_pos_pd = isam_0vel_pos.result().result_pd
        isam_0vel_pos_pd.to_csv("results/results_fgo_zupt_pos.csv", index=False)
    if not read_zupt_pos_step_from_csv:
        isam_0vel_pos_step_pd = isam_0vel_pos_step.result().result_pd
        isam_0vel_pos_step_pd.to_csv(
            "results/results_fgo_zupt_pos_step.csv", index=False
        )

    return (
        isam_0vel_pd,
        isam_0vel_step_pd,
        isam_0vel_pos_pd,
        isam_0vel_pos_step_pd,
    )


def plot_isam_trajectories_with_gt(trajectory_right, trajectory_left, position_gt):
    fig, axs = plt.subplots(1, 2, figsize=(10, 4))

    # Left foot trajectory
    axs[0].plot(
        position_gt["pos_left_y"],
        position_gt["pos_left_x"],
        label="Left Foot Ground Truth",
    )
    axs[0].plot(trajectory_left[:, 1], trajectory_left[:, 0], label="Left Est. Foot")
    axs[0].set_xlabel("X Position (m)")
    axs[0].set_ylabel("Y Position (m)")
    axs[0].legend()
    axs[0].set_title("Left Foot Trajectory")
    axs[0].grid()
    axs[0].set_xlim(-2, 3.7)  # Set x-axis limit
    axs[0].set_ylim(-0.7, 4.6)  # Set y-axis limit

    # Right foot trajectory
    axs[1].plot(
        position_gt["pos_right_y"],
        position_gt["pos_right_x"],
        label="Right Foot Ground Truth",
    )
    axs[1].plot(trajectory_right[:, 1], trajectory_right[:, 0], label="Right Est. Foot")
    axs[1].set_xlabel("X Position (m)")
    axs[1].set_ylabel("Y Position (m)")
    axs[1].legend()
    axs[1].set_title("Right Foot Trajectory")
    axs[1].grid()
    axs[1].set_xlim(-2, 3.7)  # Set x-axis limit
    axs[1].set_ylim(-0.7, 4.6)  # Set y-axis limit

    plt.tight_layout()
    plt.savefig(
        "figure/isam_foot_trajectories_plot.eps", format="eps", transparent=False
    )


def trajectory_from_isam(estimator_pd: pd.DataFrame) -> tuple:
    trajectory_right = estimator_pd[
        ["pos_right_x", "pos_right_y", "pos_right_z"]
    ].to_numpy()
    trajectory_left = estimator_pd[
        ["pos_left_x", "pos_left_y", "pos_left_z"]
    ].to_numpy()

    return np.array(trajectory_right), np.array(trajectory_left)


def plot_ekf_trajectories_with_gt(ekf_pd, position_gt):
    fig, axs = plt.subplots(1, 2, figsize=(10, 4))

    # Left foot trajectory
    axs[0].plot(
        position_gt["pos_left_y"],
        position_gt["pos_left_x"],
        label="Left Foot Ground Truth",
    )
    axs[0].plot(
        ekf_pd["pos_left_y"],
        ekf_pd["pos_left_x"],
        label="Left Est. Foot",
    )
    axs[0].set_xlabel("X Position (m)")
    axs[0].set_ylabel("Y Position (m)")
    axs[0].legend()
    axs[0].set_title("Left Foot Trajectory")
    axs[0].grid()
    axs[0].set_xlim(-2, 3.7)  # Set x-axis limit
    axs[0].set_ylim(-0.7, 4.6)  # Set y-axis limit

    # Right foot trajectory
    axs[1].plot(
        position_gt["pos_right_y"],
        position_gt["pos_right_x"],
        label="Right Foot Ground Truth",
    )
    axs[1].plot(
        ekf_pd["pos_right_y"],
        ekf_pd["pos_right_x"],
        label="Right Est. Foot",
    )
    axs[1].set_xlabel("X Position (m)")
    axs[1].set_ylabel("Y Position (m)")
    axs[1].legend()
    axs[1].set_title("Right Foot Trajectory")
    axs[1].grid()
    axs[1].set_xlim(-2, 3.7)  # Set x-axis limit
    axs[1].set_ylim(-0.7, 4.6)  # Set y-axis limit

    plt.tight_layout()
    plt.savefig(
        "figure/ekf_foot_trajectories_plot.eps", format="eps", transparent=False
    )


def compute_horizontal_errors_pd(df: pd.DataFrame, position_gt: pd.DataFrame):
    """Compute horizontal errors"""

    merged_df = pd.merge(df, position_gt, on="time_index", suffixes=("", "_gt"))
    # Compute horizontal errors
    errors_right = np.sqrt(
        (merged_df["pos_right_x"] - merged_df["pos_right_x_gt"]) ** 2
        + (merged_df["pos_right_y"] - merged_df["pos_right_y_gt"]) ** 2
    )
    errors_left = np.sqrt(
        (merged_df["pos_left_x"] - merged_df["pos_left_x_gt"]) ** 2
        + (merged_df["pos_left_y"] - merged_df["pos_left_y_gt"]) ** 2
    )

    return errors_right.to_numpy(), errors_left.to_numpy()


def compute_vertical_errors_pd(df: pd.DataFrame, position_gt: pd.DataFrame):
    # Merge estimator_df with position_gt on time_index
    merged_df = pd.merge(df, position_gt, on="time_index", suffixes=("", "_gt"))

    # Compute vertical errors
    errors_right = np.abs(merged_df["pos_right_z"] - merged_df["pos_right_z_gt"])
    errors_left = np.abs(merged_df["pos_left_z"] - merged_df["pos_left_z_gt"])

    return errors_right.to_numpy(), errors_left.to_numpy()


def plot_cdf_for_all(
    ekf_0vel: pd.DataFrame,
    ekf_0vel_step: pd.DataFrame,
    ekf_0vel_pos: pd.DataFrame,
    ekf_0vel_pos_step: pd.DataFrame,
    isam_0vel_pd: pd.DataFrame,
    isam_0vel_step_pd: pd.DataFrame,
    isam_0vel_pos_pd: pd.DataFrame,
    isam_0vel_pos_step_pd: pd.DataFrame,
    position_gt: pd.DataFrame,
):

    # Get the time indices
    timeInd_0vel = isam_0vel_pd["time_index"]

    # Filter other dataframes to match the keys in measurment epochs
    filtered_ekf_0vel = ekf_0vel[
        ekf_0vel["time_index"].isin(isam_0vel_pd["time_index"])
    ]
    filtered_ekf_0vel_step = ekf_0vel_step[
        ekf_0vel_step["time_index"].isin(isam_0vel_step_pd["time_index"])
    ]
    filtered_ekf_0vel_pos = ekf_0vel_pos[
        ekf_0vel_pos["time_index"].isin(isam_0vel_pos_pd["time_index"])
    ]
    filtered_ekf_0vel_pos_step = ekf_0vel_pos_step[
        ekf_0vel_pos_step["time_index"].isin(isam_0vel_pos_step_pd["time_index"])
    ]

    # Compute errors for all EKF methods
    ekf_errors_right_0vel, ekf_errors_left_0vel = compute_horizontal_errors_pd(
        filtered_ekf_0vel, position_gt
    )
    ekf_errors_right_step, ekf_errors_left_step = compute_horizontal_errors_pd(
        filtered_ekf_0vel_step, position_gt
    )
    ekf_errors_right_pos, ekf_errors_left_pos = compute_horizontal_errors_pd(
        filtered_ekf_0vel_pos, position_gt
    )
    ekf_errors_right_pos_step, ekf_errors_left_pos_step = compute_horizontal_errors_pd(
        filtered_ekf_0vel_pos_step, position_gt
    )

    # Compute errors for all ISAM methods
    isam_errors_right_0vel, isam_errors_left_0vel = compute_horizontal_errors_pd(
        isam_0vel_pd, position_gt
    )
    isam_errors_right_0vel_step, isam_errors_left_0vel_step = (
        compute_horizontal_errors_pd(isam_0vel_step_pd, position_gt)
    )
    isam_errors_right_pos, isam_errors_left_pos = compute_horizontal_errors_pd(
        isam_0vel_pos_pd, position_gt
    )
    isam_errors_right_pos_step, isam_errors_left_pos_step = (
        compute_horizontal_errors_pd(isam_0vel_pos_step_pd, position_gt)
    )

    # Prepare figure for CDF plots
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    def plot_cdf(ax, errors, label, color):
        """Helper function to plot CDF."""
        sorted_errors = np.sort(errors)
        cdf_values = np.linspace(0, 1, len(sorted_errors))
        ax.plot(sorted_errors, cdf_values, label=label, color=color)

    # Left foot CDF
    plot_cdf(axes[0], ekf_errors_left_0vel, "EKF-ZUPT", "blue")
    plot_cdf(axes[0], ekf_errors_left_step, "EKF-ZUPT-Step", "black")
    plot_cdf(axes[0], ekf_errors_left_pos, "EKF-ZUPT-Pos", "green")
    plot_cdf(axes[0], ekf_errors_left_pos_step, "EKF-ZUPT-Pos-Step", "cyan")
    plot_cdf(axes[0], isam_errors_left_0vel, "FGO-ZUPT", "red")
    plot_cdf(axes[0], isam_errors_left_0vel_step, "FGO-ZUPT-Step", "purple")
    plot_cdf(axes[0], isam_errors_left_pos, "FGO-ZUPT-Pos", "magenta")
    plot_cdf(axes[0], isam_errors_left_pos_step, "FGO-ZUPT-Pos-Step", "orange")
    axes[0].set_title("CDF of Horizontal Errors (Left Foot)")
    axes[0].set_xlabel("Error (m)")
    axes[0].set_ylabel("CDF")
    axes[0].grid()
    axes[0].legend()

    # Right foot CDF
    plot_cdf(axes[1], ekf_errors_right_0vel, "EKF-ZUPT", "blue")
    plot_cdf(axes[1], ekf_errors_right_step, "EKF-ZUPT-Step", "black")
    plot_cdf(axes[1], ekf_errors_right_pos, "EKF-ZUPT-Pos", "green")
    plot_cdf(axes[1], ekf_errors_right_pos_step, "EKF-ZUPT-Pos-Step", "cyan")
    plot_cdf(axes[1], isam_errors_right_0vel, "FGO-ZUPT", "red")
    plot_cdf(axes[1], isam_errors_right_0vel_step, "FGO-ZUPT-Step", "purple")
    plot_cdf(axes[1], isam_errors_right_pos, "FGO-ZUPT-Pos", "magenta")
    plot_cdf(axes[1], isam_errors_right_pos_step, "FGO-ZUPT-Pos-Step", "orange")
    axes[1].set_title("CDF of Horizontal Errors (Right Foot)")
    axes[1].set_xlabel("Error (m)")
    axes[1].set_ylabel("CDF")
    axes[1].grid()
    axes[1].legend()

    plt.tight_layout()
    plt.savefig("figure/cdf_plot.eps", format="eps", transparent=False)


def print_error_statistics(
    ekf_0vel: pd.DataFrame,
    ekf_0vel_step: pd.DataFrame,
    ekf_0vel_pos: pd.DataFrame,
    ekf_0vel_pos_step: pd.DataFrame,
    isam_0vel_pd: pd.DataFrame,
    isam_0vel_step_pd: pd.DataFrame,
    isam_0vel_pos_pd: pd.DataFrame,
    isam_0vel_pos_step_pd: pd.DataFrame,
    position_gt: pd.DataFrame,
):
    # Compute errors for all EKF methods
    ekf_errors_right_0vel_h, ekf_errors_left_0vel_h = compute_horizontal_errors_pd(
        ekf_0vel, position_gt
    )
    ekf_errors_right_step_h, ekf_errors_left_step_h = compute_horizontal_errors_pd(
        ekf_0vel_step, position_gt
    )
    ekf_errors_right_pos_h, ekf_errors_left_pos_h = compute_horizontal_errors_pd(
        ekf_0vel_pos, position_gt
    )
    ekf_errors_right_pos_step_h, ekf_errors_left_pos_step_h = (
        compute_horizontal_errors_pd(ekf_0vel_pos_step, position_gt)
    )

    # Compute errors for all ISAM methods
    isam_errors_right_0vel_h, isam_errors_left_0vel_h = compute_horizontal_errors_pd(
        isam_0vel_pd, position_gt
    )
    isam_errors_right_0vel_step_h, isam_errors_left_0vel_step_h = (
        compute_horizontal_errors_pd(isam_0vel_step_pd, position_gt)
    )
    isam_errors_right_pos_h, isam_errors_left_pos_h = compute_horizontal_errors_pd(
        isam_0vel_pos_pd, position_gt
    )
    isam_errors_right_pos_step_h, isam_errors_left_pos_step_h = (
        compute_horizontal_errors_pd(isam_0vel_pos_step_pd, position_gt)
    )

    # Compute vertical errors
    ekf_errors_right_0vel_v, ekf_errors_left_0vel_v = compute_vertical_errors_pd(
        ekf_0vel, position_gt
    )
    ekf_errors_right_step_v, ekf_errors_left_step_v = compute_vertical_errors_pd(
        ekf_0vel_step, position_gt
    )
    ekf_errors_right_pos_v, ekf_errors_left_pos_v = compute_vertical_errors_pd(
        ekf_0vel_pos, position_gt
    )
    ekf_errors_right_pos_step_v, ekf_errors_left_pos_step_v = (
        compute_vertical_errors_pd(ekf_0vel_pos_step, position_gt)
    )

    isam_errors_right_0vel_v, isam_errors_left_0vel_v = compute_vertical_errors_pd(
        isam_0vel_pd, position_gt
    )
    isam_errors_right_0vel_step_v, isam_errors_left_0vel_step_v = (
        compute_vertical_errors_pd(isam_0vel_step_pd, position_gt)
    )
    isam_errors_right_pos_v, isam_errors_left_pos_v = compute_vertical_errors_pd(
        isam_0vel_pos_pd, position_gt
    )
    isam_errors_right_pos_step_v, isam_errors_left_pos_step_v = (
        compute_vertical_errors_pd(isam_0vel_pos_step_pd, position_gt)
    )

    # Function to compute statistics
    def compute_statistics(errors):
        mean = np.mean(errors)
        rms = np.sqrt(np.mean(errors**2))
        max_error = np.max(errors)
        q90 = np.percentile(errors, 90)
        q95 = np.percentile(errors, 95)
        q99 = np.percentile(errors, 99)
        return mean, rms, max_error, q90, q95, q99

    # Prepare table headers
    methods = [
        "EKF-ZUPT",
        "EKF-ZUPT-Step",
        "EKF-ZUPT-Pos",
        "EKF-ZUPT-Pos-Step",
        "FGO-ZUPT",
        "FGO-ZUPT-Step",
        "FGO-ZUPT-Pos",
        "FGO-ZUPT-Pos-Step",
    ]
    horizontal_errors_right = [
        ekf_errors_right_0vel_h,
        ekf_errors_right_step_h,
        ekf_errors_right_pos_h,
        ekf_errors_right_pos_step_h,
        isam_errors_right_0vel_h,
        isam_errors_right_0vel_step_h,
        isam_errors_right_pos_h,
        isam_errors_right_pos_step_h,
    ]
    horizontal_errors_left = [
        ekf_errors_left_0vel_h,
        ekf_errors_left_step_h,
        ekf_errors_left_pos_h,
        ekf_errors_left_pos_step_h,
        isam_errors_left_0vel_h,
        isam_errors_left_0vel_step_h,
        isam_errors_left_pos_h,
        isam_errors_left_pos_step_h,
    ]
    vertical_errors_right = [
        ekf_errors_right_0vel_v,
        ekf_errors_right_step_v,
        ekf_errors_right_pos_v,
        ekf_errors_right_pos_step_v,
        isam_errors_right_0vel_v,
        isam_errors_right_0vel_step_v,
        isam_errors_right_pos_v,
        isam_errors_right_pos_step_v,
    ]
    vertical_errors_left = [
        ekf_errors_left_0vel_v,
        ekf_errors_left_step_v,
        ekf_errors_left_pos_v,
        ekf_errors_left_pos_step_v,
        isam_errors_left_0vel_v,
        isam_errors_left_0vel_step_v,
        isam_errors_left_pos_v,
        isam_errors_left_pos_step_v,
    ]

    # Prepare results as list of dicts
    rows_h = []
    rows_v = []
    for method, h_right, h_left, v_right, v_left in zip(
        methods,
        horizontal_errors_right,
        horizontal_errors_left,
        vertical_errors_right,
        vertical_errors_left,
    ):
        # Horizontal
        mean_r, rms_r, max_r, q90_r, q95_r, q99_r = compute_statistics(h_right)
        mean_l, rms_l, max_l, q90_l, q95_l, q99_l = compute_statistics(h_left)
        # Vertical
        vmean_r, vrms_r, vmax_r, vq90_r, vq95_r, vq99_r = compute_statistics(v_right)
        vmean_l, vrms_l, vmax_l, vq90_l, vq95_l, vq99_l = compute_statistics(v_left)

        row_h = {
            "Method": method,
            "H-Err R Mean": mean_r,
            "H-Err R RMS": rms_r,
            "H-Err R Max": max_r,
            "H-Err L Mean": mean_l,
            "H-Err L RMS": rms_l,
            "H-Err L Max": max_l,
            "H-Err R 90%": q90_r,
            "H-Err R 95%": q95_r,
            "H-Err R 99%": q99_r,
            "H-Err L 90%": q90_l,
            "H-Err L 95%": q95_l,
            "H-Err L 99%": q99_l,
        }
        row_v = {
            "Method": method,
            "V-Err R Mean": vmean_r,
            "V-Err R RMS": vrms_r,
            "V-Err R Max": vmax_r,
            "V-Err L Mean": vmean_l,
            "V-Err L RMS": vrms_l,
            "V-Err L Max": vmax_l,
            "V-Err R 90%": vq90_r,
            "V-Err R 95%": vq95_r,
            "V-Err R 99%": vq99_r,
            "V-Err L 90%": vq90_l,
            "V-Err L 95%": vq95_l,
            "V-Err L 99%": vq99_l,
        }
        rows_h.append(row_h)
        rows_v.append(row_v)

    # Save to CSV
    df_stats = pd.DataFrame(rows_h)
    df_stats.to_csv(
        "results/statistics_report_hor.csv", index=False, float_format="%.3f"
    )

    df_stats_v = pd.DataFrame(rows_v)
    df_stats_v.to_csv(
        "results/statistics_report_vert.csv", index=False, float_format="%.3f"
    )


def plot_bias_with_cov(
    biases_right,
    biases_left,
    biases_std_right,
    biases_std_left,
    plot_type: str,
    est_type: str,
):
    """
    Plot biases (accelerometer or gyroscope) with ± standard deviation.
    Each figure consists of a 3x2 subplot for the right and left foot biases.
    """

    fig, axs = plt.subplots(3, 2, figsize=(15, 10))

    # Titles for subplots
    axis_labels = ["X", "Y", "Z"]
    foot_labels = ["Right", "Left"]

    for i in range(3):  # Loop through X, Y, Z biases
        for j in range(2):  # Loop through Right (0) and Left (1) foot
            # Select data
            biases = biases_right if j == 0 else biases_left
            biases_std = biases_std_right if j == 0 else biases_std_left

            # Compute standard deviation
            std_dev = biases_std[:, i]

            # Plot bias
            axs[i, j].plot(biases[:, i], label=f"Bias {axis_labels[i]}", color="b")
            mean_val = np.mean(biases[:, i])

            # Plot +STD (upper bound)
            axs[i, j].plot(
                mean_val + std_dev,
                label=f"Mean+STD {axis_labels[i]}",
                linestyle="dashed",
                color="g",
            )

            # Plot -STD (lower bound)
            axs[i, j].plot(
                mean_val - std_dev,
                label=f"Mean-STD {axis_labels[i]}",
                linestyle="dashed",
                color="r",
            )

            # Labels and formatting
            axs[i, j].set_title(f"{foot_labels[j]} Foot Bias {axis_labels[i]}")
            axs[i, j].legend()
            axs[i, j].grid()

    plt.tight_layout()
    plt.savefig(f"figure/{est_type}_bias_plot_{plot_type}.png")


def plot_all_biases(estimator_pd: pd.DataFrame, est_type: str) -> None:
    # Extract biases and their standard deviations from the DataFrame
    acc_biases_right = estimator_pd[
        ["acc_right_x", "acc_right_y", "acc_right_z"]
    ].to_numpy()
    acc_biases_left = estimator_pd[
        ["acc_left_x", "acc_left_y", "acc_left_z"]
    ].to_numpy()
    gyro_biases_right = estimator_pd[
        ["gyro_right_x", "gyro_right_y", "gyro_right_z"]
    ].to_numpy()
    gyro_biases_left = estimator_pd[
        ["gyro_left_x", "gyro_left_y", "gyro_left_z"]
    ].to_numpy()

    acc_biases_std_right = estimator_pd[
        ["acc_right_x_std", "acc_right_y_std", "acc_right_z_std"]
    ].to_numpy()
    acc_biases_std_left = estimator_pd[
        ["acc_left_x_std", "acc_left_y_std", "acc_left_z_std"]
    ].to_numpy()
    gyro_biases_std_right = estimator_pd[
        ["gyro_right_x_std", "gyro_right_y_std", "gyro_right_z_std"]
    ].to_numpy()
    gyro_biases_std_left = estimator_pd[
        ["gyro_left_x_std", "gyro_left_y_std", "gyro_left_z_std"]
    ].to_numpy()

    plot_bias_with_cov(
        acc_biases_right,
        acc_biases_left,
        acc_biases_std_right,
        acc_biases_std_left,
        "acc",
        est_type,
    )
    plot_bias_with_cov(
        gyro_biases_right,
        gyro_biases_left,
        gyro_biases_std_right,
        gyro_biases_std_left,
        "gyro",
        est_type,
    )


def plot_foot_distance(
    ekf_0vel: pd.DataFrame,
    ekf_0vel_step: pd.DataFrame,
    ekf_0vel_pos: pd.DataFrame,
    ekf_0vel_pos_step: pd.DataFrame,
    isam_0vel_pd: pd.DataFrame,
    isam_0vel_pos_pd: pd.DataFrame,
    isam_0vel_pos_step_pd: pd.DataFrame,
    position_gt: pd.DataFrame,
) -> None:

    timeInd_0vel = isam_0vel_pd["time_index"]
    position_gt_reduced = position_gt[position_gt["time_index"].isin(timeInd_0vel)]

    def compute_foot_distance(df: pd.DataFrame) -> np.ndarray:
        foot_distances = np.linalg.norm(
            df[["pos_right_x", "pos_right_y"]].to_numpy()
            - df[["pos_left_x", "pos_left_y"]].to_numpy(),
            axis=1,
        )
        return foot_distances

    gt_foot_dist = compute_foot_distance(position_gt_reduced)

    # Compute foot distances for all EKF methods
    ekf_0vel_reduce = ekf_0vel[ekf_0vel["time_index"].isin(timeInd_0vel)]
    ekf_foot_dist_0vel = compute_foot_distance(ekf_0vel_reduce)
    ekf_0vel_step_reduce = ekf_0vel_step[ekf_0vel_step["time_index"].isin(timeInd_0vel)]
    ekf_foot_dist_step = compute_foot_distance(ekf_0vel_step_reduce)
    ekf_0vel_pos_reduce = ekf_0vel_pos[ekf_0vel_pos["time_index"].isin(timeInd_0vel)]
    ekf_foot_dist_pos = compute_foot_distance(ekf_0vel_pos_reduce)
    ekf_0vel_pos_step_reduce = ekf_0vel_pos_step[
        ekf_0vel_pos_step["time_index"].isin(timeInd_0vel)
    ]
    ekf_foot_dist_pos_step = compute_foot_distance(ekf_0vel_pos_step_reduce)

    # Compute foot distances for all ISAM methods
    isam_foot_dist_0vel = compute_foot_distance(isam_0vel_pd)

    isam_0vel_pos_pd = isam_0vel_pos_pd[
        isam_0vel_pos_pd["time_index"].isin(timeInd_0vel)
    ]
    isam_foot_dist_pos = compute_foot_distance(isam_0vel_pos_pd)

    isam_0vel_pos_step_pd = isam_0vel_pos_step_pd[
        isam_0vel_pos_step_pd["time_index"].isin(timeInd_0vel)
    ]
    isam_foot_dist_pos_step = compute_foot_distance(isam_0vel_pos_step_pd)

    time = np.array(timeInd_0vel) * 1 / 60

    # Plot them in a single figure (no subplot layout)
    plt.figure(figsize=(6, 4))
    plt.plot(time, ekf_foot_dist_0vel, label="EKF-ZUPT", color="blue")
    plt.plot(time, ekf_foot_dist_step, label="EKF-ZUPT-Step", color="black")
    plt.plot(time, ekf_foot_dist_pos, label="EKF-ZUPT-Pos", color="green")
    plt.plot(time, ekf_foot_dist_pos_step, label="EKF-ZUPT-Pos-Step", color="cyan")
    plt.plot(time, isam_foot_dist_0vel, label="FGO-ZUPT", color="red")
    plt.plot(time, isam_foot_dist_pos, label="FGO-ZUPT-Pos", color="magenta")
    plt.plot(time, isam_foot_dist_pos_step, label="FGO-ZUPT-Pos-Step", color="orange")
    plt.plot(time, gt_foot_dist, label="Ground Truth", color="black")
    plt.title("Foot Distance")
    plt.xlabel("Time (s)")
    plt.ylabel("Distance (m)")
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.savefig("figure/foot_distance_plot.eps")


def compute_gt_velocity(position_gt: pd.DataFrame) -> pd.DataFrame:
    """Compute ground truth velocity from ground truth positions."""
    dt = 1 / 60

    velocity_gt = position_gt[["time_index"]].copy()

    pos_columns = [
        "pos_right_x",
        "pos_right_y",
        "pos_right_z",
        "pos_left_x",
        "pos_left_y",
        "pos_left_z",
    ]

    # Create velocity column names
    vel_columns = [pos_col.replace("pos_", "vel_") for pos_col in pos_columns]

    # Initialize velocity_gt DataFrame with NaN values
    velocity_gt = position_gt[["time_index"]].copy()
    for vel_col in vel_columns:
        velocity_gt[vel_col] = float("nan")  # Initialize empty velocity columns

    for pos_col, vel_col in zip(pos_columns, vel_columns):

        # Compute forward difference for first timestamp
        velocity_gt.loc[0, vel_col] = (
            position_gt.loc[1, pos_col] - position_gt.loc[0, pos_col]
        ) / dt

        # Compute central difference for middle timestamps
        central_diff = (
            position_gt[pos_col].shift(-1) - position_gt[pos_col].shift(1)
        ) / (2 * dt)
        velocity_gt.loc[1 : len(position_gt) - 2, vel_col] = central_diff[
            1 : len(position_gt) - 1
        ].values

        # Compute backward difference for the last timestamp
        velocity_gt.loc[len(velocity_gt) - 1, vel_col] = (
            position_gt.loc[len(position_gt) - 1, pos_col]
            - position_gt.loc[len(position_gt) - 2, pos_col]
        ) / dt

    return velocity_gt


def plot_velocity_vs_gt(
    velocity_est: pd.DataFrame,
    velocity_gt: pd.DataFrame,
    imu_right_pd: pd.DataFrame,
    imu_left_pd: pd.DataFrame,
    est_type: str,
):
    """Plot estimated velocity against ground truth velocity."""
    # Titles for subplots
    axis_labels = ["X", "Y", "Z"]
    foot_labels = ["Right", "Left"]

    dt = 1 / 60  # Define time scale

    # Create a 3x2 subplot layout
    fig = make_subplots(
        rows=3,
        cols=2,
        subplot_titles=[
            f"{foot} Foot Velocity {axis}"
            for axis in axis_labels
            for foot in foot_labels
        ],
        horizontal_spacing=0.1,
        vertical_spacing=0.1,
    )

    for i in range(3):  # Loop through X, Y, Z velocities
        for j in range(2):  # Loop through Right (0) and Left (1) foot
            row, col = i + 1, j + 1  # Subplot position

            # Select data
            vel_gt_col = f"vel_{foot_labels[j].lower()}_{axis_labels[i].lower()}"
            vel_est_col = f"vel_{foot_labels[j].lower()}_{axis_labels[i].lower()}"
            imu_pd = imu_right_pd if j == 0 else imu_left_pd

            # Plot ground truth velocity
            fig.add_trace(
                go.Scatter(
                    x=velocity_gt["time_index"] * dt,
                    y=velocity_gt[vel_gt_col],
                    mode="lines",
                    name="GT",
                    line=dict(color="blue"),
                ),
                row=row,
                col=col,
            )

            # Plot estimated velocity
            fig.add_trace(
                go.Scatter(
                    x=velocity_est["time_index"] * dt,
                    y=velocity_est[vel_est_col],
                    mode="lines",
                    name=f"{est_type} Est.",
                    line=dict(color="red"),
                ),
                row=row,
                col=col,
            )

            # Plot IMU zero velocity indicator
            fig.add_trace(
                go.Scatter(
                    x=imu_pd["time_index"] * dt,
                    y=imu_pd["is_zero_vel"] * np.max(velocity_gt[vel_gt_col]),
                    mode="lines",
                    name="IMU Zero Vel",
                    line=dict(color="gray", dash="dash"),
                    opacity=0.5,
                ),
                row=row,
                col=col,
            )

    # Update layout
    fig.update_layout(
        title_text=f"{est_type} Velocity vs Ground Truth",
        showlegend=True,
        autosize=True,
    )

    fig.update_layout(
        xaxis=dict(scaleanchor="x"),
        xaxis2=dict(scaleanchor="x"),
        xaxis3=dict(scaleanchor="x"),
        xaxis4=dict(scaleanchor="x"),
        xaxis5=dict(scaleanchor="x"),
        xaxis6=dict(scaleanchor="x"),
        yaxis=dict(scaleanchor="y"),
        yaxis2=dict(scaleanchor="y"),
        yaxis3=dict(scaleanchor="y"),
        yaxis4=dict(scaleanchor="y"),
        yaxis5=dict(scaleanchor="y"),
        yaxis6=dict(scaleanchor="y"),
    )

    # Save as interactive HTML
    fig.write_html(f"figure/{est_type}_Velocity.html")

    # Show plot
    fig.show()


def plot_velocity_errors(
    velocity_est: pd.DataFrame, velocity_gt: pd.DataFrame, est_type: str
):
    """Plot velocity errors against ground truth velocity."""
    # Calculate velocity errors
    velocity_errors = velocity_est.copy()
    for col in [
        "vel_right_x",
        "vel_right_y",
        "vel_right_z",
        "vel_left_x",
        "vel_left_y",
        "vel_left_z",
    ]:
        velocity_errors[col] = velocity_est[col] - velocity_gt[col]

    # Plot all velocity error components for right and left in 3 by 2 subplots
    fig, axs = plt.subplots(3, 2, figsize=(15, 10))
    # Titles for subplots
    axis_labels = ["X", "Y", "Z"]
    foot_labels = ["Right", "Left"]

    dt = 1 / 60  # Define time scale

    for i in range(3):  # Loop through X, Y, Z velocities
        for j in range(2):  # Loop through Right (0) and Left (1) foot
            # Select data
            vel_err_col = f"vel_{foot_labels[j].lower()}_{axis_labels[i].lower()}"

            # Plot velocity errors
            axs[i, j].plot(
                velocity_errors["time_index"] * dt,
                velocity_errors[vel_err_col],
                label=f"{est_type} Error",
                color="r",
            )

            # Labels and formatting
            axs[i, j].set_title(
                f"{foot_labels[j]} Foot Velocity Error {axis_labels[i]}"
            )
            axs[i, j].legend()
            axs[i, j].grid()

    plt.tight_layout()
    plt.savefig(f"figure/{est_type}_Velocity_Error.png")


def plot_velocity_all(
    isam_0vel_pd,
    isam_0vel_pos_pd,
    isam_0vel_pos_step_pd,
    position_gt,
    imu_right_pd,
    imu_left_pd,
):

    plot_velocity_vs_gt(
        isam_0vel_pd, position_gt, imu_right_pd, imu_left_pd, "FGO_ZUPT"
    )
    plot_velocity_vs_gt(
        isam_0vel_pos_pd, position_gt, imu_right_pd, imu_left_pd, "FGO_ZUPT_Pos"
    )
    plot_velocity_vs_gt(
        isam_0vel_pos_step_pd,
        position_gt,
        imu_right_pd,
        imu_left_pd,
        "FGO_ZUPT_Pos_Foot",
    )


def main():
    imu_right_file = "measData/imuMeasRight.csv"
    imu_left_file = "measData/imuMeasLeft.csv"
    position_meas_file = "measData/positionMeas.csv"
    position_gt_file = "measData/positionGt.csv"

    # Load EKF results
    ekf_0vel_file = "results/results_ekf_zupt.csv"
    ekf_0vel_step_file = "results/results_ekf_zupt_step.csv"
    ekf_0vel_pos_file = "results/results_ekf_zupt_pos.csv"
    ekf_0vel_pos_step_file = "results/results_ekf_zupt_pos_step.csv"

    ekf_0vel = pd.read_csv(ekf_0vel_file)
    ekf_0vel_step = pd.read_csv(ekf_0vel_step_file)
    ekf_0vel_pos = pd.read_csv(ekf_0vel_pos_file)
    ekf_0vel_pos_step = pd.read_csv(ekf_0vel_pos_step_file)

    position_meas = pd.read_csv(position_meas_file)
    position_gt = pd.read_csv(position_gt_file)

    print("Plotting measurement VS GT...")
    plot_measurement_errors(position_meas, position_gt)

    position_gt = position_gt.merge(
        compute_gt_velocity(position_gt), on="time_index", how="left"
    )

    print("Obtaining GTSAM results...")
    isam_0vel_pd, isam_0vel_step_pd, isam_0vel_pos_pd, isam_0vel_pos_step_pd = (
        obtain_gtsam_restuls(
            imu_right_file,
            imu_left_file,
            position_meas_file,
            read_zupt_from_csv=True,
            read_zupt_step_from_csv=True,
            read_zupt_pos_from_csv=True,
            read_zupt_pos_step_from_csv=True,
        )
    )
    timeInd_0vel_pos_step = isam_0vel_pos_step_pd["time_index"]
    # Select the first 100 time indices

    trajectory_right, trajectory_left = trajectory_from_isam(isam_0vel_pos_step_pd)
    # Filter position_gt to match the keys in timeInd_0vel_pos_step
    filtered_position_gt = position_gt[
        position_gt["time_index"].isin(timeInd_0vel_pos_step)
    ]

    print("Plotting FGO trajectories.")
    plot_isam_trajectories_with_gt(
        trajectory_right, trajectory_left, filtered_position_gt
    )
    filtered_ekf_0vel_pos_step = ekf_0vel_pos_step[
        ekf_0vel_pos_step["time_index"].isin(timeInd_0vel_pos_step)
    ]

    print("Plotting EKF trajectories.")
    plot_ekf_trajectories_with_gt(filtered_ekf_0vel_pos_step, filtered_position_gt)

    print("Plotting CDF for all methods.")
    plot_cdf_for_all(
        ekf_0vel,
        ekf_0vel_step,
        ekf_0vel_pos,
        ekf_0vel_pos_step,
        isam_0vel_pd,
        isam_0vel_step_pd,
        isam_0vel_pos_pd,
        isam_0vel_pos_step_pd,
        position_gt,
    )

    print("Plotting velocity for all methods.")
    imu_right_pd = pd.read_csv(imu_right_file)
    imu_left_pd = pd.read_csv(imu_left_file)
    plot_velocity_all(
        isam_0vel_pd,
        isam_0vel_pos_pd,
        isam_0vel_pos_step_pd,
        position_gt,
        imu_right_pd,
        imu_left_pd,
    )

    print("Plotting biases for all methods.")
    plot_all_biases(estimator_pd=isam_0vel_pd, est_type="FGO_ZUPT")
    plot_all_biases(estimator_pd=isam_0vel_pos_pd, est_type="FGO_ZUPT_Pos")
    plot_all_biases(estimator_pd=isam_0vel_pos_step_pd, est_type="FGO_ZUPT_Pos_Foot")

    print("Plotting foot distance.")
    plot_foot_distance(
        ekf_0vel,
        ekf_0vel_step,
        ekf_0vel_pos,
        ekf_0vel_pos_step,
        isam_0vel_pd,
        isam_0vel_pos_pd,
        isam_0vel_pos_step_pd,
        position_gt,
    )

    print("Printing error statistics.")
    print_error_statistics(
        ekf_0vel,
        ekf_0vel_step,
        ekf_0vel_pos,
        ekf_0vel_pos_step,
        isam_0vel_pd,
        isam_0vel_step_pd,
        isam_0vel_pos_pd,
        isam_0vel_pos_step_pd,
        position_gt,
    )
    print("Done! See results in the 'results' folder.")


if __name__ == "__main__":
    main()
