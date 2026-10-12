#!/usr/bin/env python3
"""Real-time plot of Xsens MTw linear acceleration and/or angular velocity.

Subscribes to either the mono topic (imu_msgs/IMUDataArray, default
"xsens_imu_data") or one-topic-per-imu (imu_msgs/IMUDataSingle, topics named
"<mtw_topic_name>_<mtwID>") and rotates each IMU's data from its local
placement frame into the NWU world frame (x-axis forward / y-axis left /
z-axis up) before plotting.

Examples:
    ros2 launch xsens_mtw_driver xsens_mtw_manager.launch.py

    # mono topic (one_topic_per_imu: false in params.yaml), all IMUs
    ./plot_acc_gyr.py

    # one-topic-per-imu (one_topic_per_imu: true in params.yaml)
    ./plot_acc_gyr.py --one-topic-per-imu --imu-id 00B493A1 --no-gyr
"""

import argparse
import threading
from collections import defaultdict, deque

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from scipy.spatial.transform import Rotation

from imu_msgs.msg import IMUData, IMUDataArray, IMUDataSingle

# Default rotation (extrinsic XYZ euler angles, radians) which aligns IMU
# data from its local placement frame to the NWU world frame
# (x-axis forward / y-axis left / z-axis up).
DEFAULT_ROTATION_XYZ = [0, np.pi / 2, -np.pi / 2]


def build_rotation(euler_xyz):
    # Intrinsic XYZ euler, inverted: matches MT Manager's sensor-to-world
    # alignment convention (verified against MT Manager on Windows).
    return Rotation.from_euler("XYZ", euler_xyz).inv()


class ImuPlotter(Node):
    def __init__(self, args):
        super().__init__("xsens_imu_plotter")

        self._plot_acc = not args.no_acc
        self._plot_gyr = not args.no_gyr
        self._window_s = args.window
        self._rotation = build_rotation(args.rotation)

        self._lock = threading.Lock()
        self._t0 = None
        self._data = defaultdict(lambda: {
            "t": deque(maxlen=args.max_samples),
            "acc": deque(maxlen=args.max_samples),
            "gyr": deque(maxlen=args.max_samples),
        })

        # Publishers use best-effort QoS (sensor_qos in xsens_mtw_manager.cpp);
        # match it or the subscription is dropped as QoS-incompatible.
        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value

        if args.one_topic_per_imu:
            # Each IMU publishes IMUDataSingle on its own topic "<topic>_<mtwID>".
            if not args.imu_id:
                self.get_logger().warn(
                    "--one-topic-per-imu requires --imu-id (repeatable); no topics subscribed.")
            for imu_id in args.imu_id or []:
                self.create_subscription(
                    IMUDataSingle, f"{args.topic}_{imu_id}", self._single_cb, sensor_qos)
        else:
            self._imu_filter = set(args.imu_id) if args.imu_id else None
            self.create_subscription(IMUDataArray, args.topic, self._array_cb, sensor_qos)

    def _single_cb(self, msg: IMUDataSingle):
        self._store(msg.imu_data)

    def _array_cb(self, msg: IMUDataArray):
        for imu in msg.imu_data:
            if self._imu_filter is None or imu.id in self._imu_filter:
                self._store(imu)

    def _store(self, imu: IMUData):
        stamp = imu.receive_stamp.sec + imu.receive_stamp.nanosec * 1e-9

        with self._lock:
            if self._t0 is None:
                self._t0 = stamp
            t = stamp - self._t0

            entry = self._data[imu.id]
            entry["t"].append(t)

            if self._plot_acc:
                acc = np.array([
                    imu.linear_acceleration.x,
                    imu.linear_acceleration.y,
                    imu.linear_acceleration.z,
                ])
                entry["acc"].append(self._rotation.apply(acc))

            if self._plot_gyr:
                gyr = np.array([
                    imu.angular_velocity.x,
                    imu.angular_velocity.y,
                    imu.angular_velocity.z,
                ])
                entry["gyr"].append(self._rotation.apply(gyr))

    def snapshot(self):
        with self._lock:
            return {
                imu_id: {
                    "t": np.array(entry["t"]),
                    "acc": np.array(entry["acc"]) if entry["acc"] else np.empty((0, 3)),
                    "gyr": np.array(entry["gyr"]) if entry["gyr"] else np.empty((0, 3)),
                }
                for imu_id, entry in self._data.items()
            }


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--topic", default="xsens_imu_data", help="topic to subscribe to (default: xsens_imu_data)")
    parser.add_argument("--one-topic-per-imu", action="store_true", help="subscribe as imu_msgs/IMUData instead of imu_msgs/IMUDataArray")
    parser.add_argument("--imu-id", action="append", help="IMU id to plot (repeatable). With --one-topic-per-imu this selects which "
                         "'<topic>_<id>' topics to subscribe to (required); otherwise it filters the mono topic (default: all IMUs)")
    parser.add_argument("--no-acc", action="store_true", help="disable acceleration plot")
    parser.add_argument("--no-gyr", action="store_true", help="disable angular velocity plot")
    parser.add_argument("--window", type=float, default=10.0, help="seconds of history shown on screen (default: 10)")
    parser.add_argument("--max-samples", type=int, default=5000, help="max buffered samples per IMU (default: 5000)")
    parser.add_argument("--rotation", type=float, nargs=3, default=DEFAULT_ROTATION_XYZ, metavar=("RX", "RY", "RZ"), help="extrinsic XYZ euler rotation (rad) applied to align local IMU frame to NWU world frame")
    args = parser.parse_args()

    if args.no_acc and args.no_gyr:
        parser.error("cannot disable both --no-acc and --no-gyr")

    rclpy.init()
    node = ImuPlotter(args)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    n_rows = int(not args.no_acc) + int(not args.no_gyr)
    fig, axes = plt.subplots(n_rows, 1, sharex=True, squeeze=False)
    axes = axes[:, 0]
    row = 0
    if not args.no_acc:
        acc_ax = axes[row]
        acc_ax.set_ylabel("linear acceleration [m/s^2]")
        row += 1
    if not args.no_gyr:
        gyr_ax = axes[row]
        gyr_ax.set_ylabel("angular velocity [rad/s]")
    axes[-1].set_xlabel("time [s]")

    axis_colors = {"x": "red", "y": "green", "z": "blue"}
    linestyles = ["-", "--", ":", "-."]
    lines = {}

    def get_lines(imu_id, linestyle):
        if imu_id not in lines:
            entry = {}
            if not args.no_acc:
                entry["acc"] = [acc_ax.plot([], [], color=axis_colors[axis], linestyle=linestyle, label=f"{imu_id} {axis}")[0]
                                 for axis in "xyz"]
            if not args.no_gyr:
                entry["gyr"] = [gyr_ax.plot([], [], color=axis_colors[axis], linestyle=linestyle, label=f"{imu_id} {axis}")[0]
                                for axis in "xyz"]
            lines[imu_id] = entry
        return lines[imu_id]

    legend_drawn = False

    def update(_frame):
        nonlocal legend_drawn
        snap = node.snapshot()
        if not snap:
            return []

        artists = []
        t_max = 0.0
        for i, (imu_id, entry) in enumerate(sorted(snap.items())):
            if entry["t"].size == 0:
                continue
            t_max = max(t_max, entry["t"][-1])
            linestyle = linestyles[i % len(linestyles)]
            imu_lines = get_lines(imu_id, linestyle)

            if not args.no_acc and entry["acc"].shape[0] > 0:
                for axis_idx, line in enumerate(imu_lines["acc"]):
                    line.set_data(entry["t"], entry["acc"][:, axis_idx])
                    artists.append(line)

            if not args.no_gyr and entry["gyr"].shape[0] > 0:
                for axis_idx, line in enumerate(imu_lines["gyr"]):
                    line.set_data(entry["t"], entry["gyr"][:, axis_idx])
                    artists.append(line)

        t_min = max(0.0, t_max - args.window)
        for ax in axes:
            ax.set_xlim(t_min, max(t_max, args.window))
            ax.relim()
            ax.autoscale_view(scalex=False)

        if not legend_drawn and lines:
            for ax in axes:
                ax.legend(loc="upper left", fontsize="small", ncol=max(1, len(lines)))
            legend_drawn = True

        return artists

    from matplotlib.animation import FuncAnimation
    anim = FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)  # noqa: F841 (keep ref alive)

    try:
        plt.tight_layout()
        plt.show()
    finally:
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
