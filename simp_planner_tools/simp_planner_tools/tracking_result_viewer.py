"""Final tracking results, displayed independently of the ROS launch process."""

import sys

import numpy as np


def save_history(history, path):
    rows = np.asarray(history, dtype=float).reshape(-1, 9)
    dx, dy = rows[:, 3] - rows[:, 1], rows[:, 4] - rows[:, 2]
    yaw_error = (rows[:, 7] - rows[:, 5] + np.pi) % (2 * np.pi) - np.pi
    rate_error = rows[:, 8] - rows[:, 6]
    np.savetxt(
        path,
        np.column_stack((rows[:, :5], dx, dy, np.hypot(dx, dy),
                         rows[:, 5:], yaw_error, rate_error)),
        delimiter=",", comments="",
        header=("time_s,target_x_m,target_y_m,actual_x_m,actual_y_m,dx_m,dy_m,error_norm_m,"
                "target_yaw_rad,target_yaw_rate_rad_s,actual_yaw_rad,actual_yaw_rate_rad_s,"
                "yaw_error_rad,yaw_rate_error_rad_s"),
    )


def create_figures(data, title):
    import matplotlib.pyplot as plt

    comparison, comparison_axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True,
                                               constrained_layout=True)
    errors, error_axes = plt.subplots(5, 1, figsize=(10, 9), sharex=True,
                                     constrained_layout=True)
    comparison.canvas.manager.set_window_title("Tracking: target and actual")
    errors.canvas.manager.set_window_title("Tracking: errors")
    comparison.suptitle(title + "\nPlanner target vs simulator (same odometry timestamp)")
    errors.suptitle(title + "\nError = actual - target; yaw error wrapped to [-180, 180)")
    time = data["time_s"]
    for axis, error_axis, target, actual, error, label, scale in zip(
        comparison_axes, error_axes,
        ("target_x_m", "target_y_m", "target_yaw_rad", "target_yaw_rate_rad_s"),
        ("actual_x_m", "actual_y_m", "actual_yaw_rad", "actual_yaw_rate_rad_s"),
        ("dx_m", "dy_m", "yaw_error_rad", "yaw_rate_error_rad_s"),
        ("x [m]", "y [m]", "yaw [deg]", "yaw rate [deg/s]"),
        (1, 1, 180 / np.pi, 180 / np.pi),
    ):
        axis.plot(time, data[target] * scale, "--", label="Planner target")
        axis.plot(time, data[actual] * scale, label="Simulator /odom")
        axis.set_ylabel(label)
        axis.legend(loc="upper right")
        error_axis.plot(time, data[error] * scale)
        error_axis.axhline(0, color="gray", linewidth=0.8)
        error_axis.set_ylabel(label)
    error_axes[-1].plot(time, data["error_norm_m"])
    error_axes[-1].set_ylabel("position error [m]")
    error_axes[-1].set_ylim(bottom=0)
    for axes in (comparison_axes, error_axes):
        for axis in axes:
            axis.grid(True, alpha=0.3)
        axes[-1].set_xlabel("time [s]; gaps = unavailable target")
    return comparison, errors


def main():
    # A fresh process avoids the Agg backend used by the periodic PNG renderer.
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt

    data = np.genfromtxt(sys.argv[1], delimiter=",", names=True, ndmin=1)
    create_figures(data, sys.argv[2] if len(sys.argv) > 2 else "Tracking results")
    plt.show()


if __name__ == "__main__":
    main()
