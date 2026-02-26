#!/usr/bin/env python3
# Copyright (c) 2025 - Franka Data Collection Node
# Collects 42-dimensional state from follower robot during teleoperation
# Saves to CSV and NPY formats
#
# Topics subscribed (under follower namespace, e.g. /franka_teleop/follower/):
#   franka_robot_state_broadcaster/robot_state  -> FrankaRobotState (joint pos, vel, torques, ext torques, EE pose)
#   franka_robot_state_broadcaster/measured_joint_states -> sensor_msgs/JointState (joint pos, vel, torque)
#
# Data format per row (42 values):
#   [0]      timestamp_ns
#   [1-7]    joint positions (q1..q7)
#   [8-14]   joint velocities (dq1..dq7)
#   [15-21]  measured torques (tau1..tau7)
#   [22-28]  external torques (tau_ext1..tau_ext7)
#   [29-31]  Cartesian position (x, y, z)
#   [32-35]  quaternion (qx, qy, qz, qw)
#   [36-41]  Cartesian velocities (vx, vy, vz, wx, wy, wz)

import os
import time
import signal
import threading
from pathlib import Path
from datetime import datetime
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState

# Try to import franka-specific messages
try:
    from franka_msgs.msg import FrankaRobotState
    FRANKA_STATE_AVAILABLE = True
except ImportError:
    FRANKA_STATE_AVAILABLE = False

NUM_JOINTS = 7
STATE_DIM = 42  # 1 + 7 + 7 + 7 + 7 + 3 + 4 + 6


def rotation_matrix_to_quaternion(R):
    """Convert 3x3 rotation matrix to quaternion [qx, qy, qz, qw]."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


class FrankaDataCollector(Node):
    """
    ROS 2 node to collect and save follower robot state data during teleoperation.

    Collects 42 values per timestep:
      - timestamp_ns (1)
      - joint positions (7)
      - joint velocities (7)
      - measured torques (7)
      - external torques (7)
      - Cartesian position xyz (3)
      - quaternion xyzw (4)
      - Cartesian velocities vx vy vz wx wy wz (6)
    """

    def __init__(self):
        super().__init__('franka_data_collector')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('follower_namespace', 'franka_teleop/follower')
        self.declare_parameter('output_dir', '/tmp/franka_data')
        self.declare_parameter('save_csv', True)
        self.declare_parameter('save_npy', True)
        self.declare_parameter('buffer_size', 100000)
        self.declare_parameter('auto_save_interval_sec', 30.0)
        self.declare_parameter('use_franka_robot_state', True)

        self.follower_ns = self.get_parameter('follower_namespace').value
        self.output_dir = Path(self.get_parameter('output_dir').value)
        self.save_csv = self.get_parameter('save_csv').value
        self.save_npy = self.get_parameter('save_npy').value
        self.buffer_size = self.get_parameter('buffer_size').value
        self.auto_save_interval = self.get_parameter('auto_save_interval_sec').value
        self.use_franka_state = self.get_parameter('use_franka_robot_state').value

        self.output_dir.mkdir(parents=True, exist_ok=True)

        # ── Internal state ────────────────────────────────────────────────────
        self.lock = threading.Lock()
        self.data_buffer = []

        # Latest values (thread-safe copies for assembling rows)
        self._joint_pos = np.zeros(NUM_JOINTS)
        self._joint_vel = np.zeros(NUM_JOINTS)
        self._measured_torques = np.zeros(NUM_JOINTS)
        self._ext_torques = np.zeros(NUM_JOINTS)
        self._cart_pos = np.zeros(3)
        self._quat = np.array([0.0, 0.0, 0.0, 1.0])   # xyzw
        self._cart_vel = np.zeros(6)

        self._franka_state_received = False
        self._joint_state_received = False

        self._session_start = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._sample_count = 0

        # ── QoS for robot state topics ─────────────────────────────────────
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscriptions ──────────────────────────────────────────────────
        ns = self.follower_ns.rstrip('/')

        # JointState from franka_robot_state_broadcaster (has position, velocity, effort = measured torque)
        measured_js_topic = f'/{ns}/franka_robot_state_broadcaster/measured_joint_states'
        self.joint_state_sub = self.create_subscription(
            JointState,
            measured_js_topic,
            self._joint_state_callback,
            best_effort_qos,
        )
        self.get_logger().info(f'Subscribing to: {measured_js_topic}')

        # FrankaRobotState (has ee_pose, external_joint_torques, O_dP_EE)
        if self.use_franka_state and FRANKA_STATE_AVAILABLE:
            robot_state_topic = f'/{ns}/franka_robot_state_broadcaster/robot_state'
            self.franka_state_sub = self.create_subscription(
                FrankaRobotState,
                robot_state_topic,
                self._franka_state_callback,
                best_effort_qos,
            )
            self.get_logger().info(f'Subscribing to: {robot_state_topic}')
        else:
            if not FRANKA_STATE_AVAILABLE:
                self.get_logger().warn(
                    'franka_msgs not available – external torques and Cartesian data will be zeros. '
                    'Make sure franka_ros2 is built and sourced.')
            self.franka_state_sub = None

        # Also subscribe to external_joint_torques topic as fallback/complement
        ext_torque_topic = f'/{ns}/franka_robot_state_broadcaster/external_joint_torques'
        self.ext_torque_sub = self.create_subscription(
            JointState,
            ext_torque_topic,
            self._ext_torque_callback,
            best_effort_qos,
        )
        self.get_logger().info(f'Subscribing to: {ext_torque_topic}')

        # ── Timer to record data rows at ~1kHz ─────────────────────────────
        self.record_timer = self.create_timer(0.001, self._record_callback)

        # ── Auto-save timer ────────────────────────────────────────────────
        if self.auto_save_interval > 0:
            self.save_timer = self.create_timer(self.auto_save_interval, self._auto_save)

        self.get_logger().info(
            f'FrankaDataCollector started\n'
            f'  Follower namespace : {self.follower_ns}\n'
            f'  Output directory   : {self.output_dir}\n'
            f'  Buffer size        : {self.buffer_size}\n'
            f'  Auto-save interval : {self.auto_save_interval}s\n'
            f'  Save CSV           : {self.save_csv}\n'
            f'  Save NPY           : {self.save_npy}\n'
        )

        # Register SIGINT/SIGTERM so we save on Ctrl+C
        signal.signal(signal.SIGINT, self._shutdown_handler)
        signal.signal(signal.SIGTERM, self._shutdown_handler)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _joint_state_callback(self, msg: JointState):
        """Measured joint states: position, velocity, effort (measured torque)."""
        n = min(len(msg.position), NUM_JOINTS)
        with self.lock:
            if n > 0:
                self._joint_pos[:n] = msg.position[:n]
            n_vel = min(len(msg.velocity), NUM_JOINTS)
            if n_vel > 0:
                self._joint_vel[:n_vel] = msg.velocity[:n_vel]
            n_eff = min(len(msg.effort), NUM_JOINTS)
            if n_eff > 0:
                self._measured_torques[:n_eff] = msg.effort[:n_eff]
            self._joint_state_received = True

    def _ext_torque_callback(self, msg: JointState):
        """External joint torques published by franka_robot_state_broadcaster."""
        n = min(len(msg.effort), NUM_JOINTS)
        with self.lock:
            if n > 0:
                self._ext_torques[:n] = msg.effort[:n]

    def _franka_state_callback(self, msg):
        """
        FrankaRobotState contains:
          O_T_EE  - 4x4 homogeneous transform (flat 16-element array, column-major)
          O_dP_EE - Cartesian velocity (6-element: vx vy vz wx wy wz)
          tau_ext_hat_filtered - external joint torques (7-element)
        """
        with self.lock:
            # Extract 4x4 EE transform (column-major, 16 elements)
            if hasattr(msg, 'o_t_ee') and len(msg.o_t_ee) == 16:
                T = np.array(msg.o_t_ee).reshape(4, 4, order='F')
                self._cart_pos = T[:3, 3].copy()
                R = T[:3, :3]
                self._quat = rotation_matrix_to_quaternion(R)
            elif hasattr(msg, 'o_t_ee_d') and len(msg.o_t_ee_d) == 16:
                T = np.array(msg.o_t_ee_d).reshape(4, 4, order='F')
                self._cart_pos = T[:3, 3].copy()
                R = T[:3, :3]
                self._quat = rotation_matrix_to_quaternion(R)

            # Cartesian velocity
            if hasattr(msg, 'o_dp_ee_d') and len(msg.o_dp_ee_d) == 6:
                self._cart_vel = np.array(msg.o_dp_ee_d)
            elif hasattr(msg, 'o_dp_ee_c') and len(msg.o_dp_ee_c) == 6:
                self._cart_vel = np.array(msg.o_dp_ee_c)

            # External torques from franka state (more filtered)
            if hasattr(msg, 'tau_ext_hat_filtered') and len(msg.tau_ext_hat_filtered) == NUM_JOINTS:
                self._ext_torques = np.array(msg.tau_ext_hat_filtered)

            self._franka_state_received = True

    def _record_callback(self):
        """Called at ~1kHz - assembles and buffers one data row."""
        ts_ns = self.get_clock().now().nanoseconds

        with self.lock:
            row = np.empty(STATE_DIM)
            row[0] = float(ts_ns)
            row[1:8] = self._joint_pos
            row[8:15] = self._joint_vel
            row[15:22] = self._measured_torques
            row[22:29] = self._ext_torques
            row[29:32] = self._cart_pos
            row[32:36] = self._quat           # qx qy qz qw
            row[36:42] = self._cart_vel
            self.data_buffer.append(row)
            self._sample_count += 1

        # Auto-flush if buffer is full
        if len(self.data_buffer) >= self.buffer_size:
            self.get_logger().info(f'Buffer full ({self.buffer_size} samples), saving...')
            self._save_data(suffix='_partial')

    def _auto_save(self):
        n = len(self.data_buffer)
        if n > 0:
            self.get_logger().info(f'Auto-save: {n} samples collected so far.')
            self._save_data(suffix='_autosave')

    def _save_data(self, suffix=''):
        """Save buffered data to disk and clear the buffer."""
        with self.lock:
            if not self.data_buffer:
                self.get_logger().info('No data to save.')
                return
            arr = np.array(self.data_buffer, dtype=np.float64)
            self.data_buffer = []

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        base_name = f'franka_follower_{self._session_start}{suffix}_{timestamp}'

        saved = []

        if self.save_npy:
            npy_path = self.output_dir / f'{base_name}.npy'
            np.save(str(npy_path), arr)
            saved.append(str(npy_path))

        if self.save_csv:
            csv_path = self.output_dir / f'{base_name}.csv'
            header = (
                'timestamp_ns,'
                'q1,q2,q3,q4,q5,q6,q7,'
                'dq1,dq2,dq3,dq4,dq5,dq6,dq7,'
                'tau1,tau2,tau3,tau4,tau5,tau6,tau7,'
                'tau_ext1,tau_ext2,tau_ext3,tau_ext4,tau_ext5,tau_ext6,tau_ext7,'
                'pos_x,pos_y,pos_z,'
                'quat_x,quat_y,quat_z,quat_w,'
                'vel_vx,vel_vy,vel_vz,vel_wx,vel_wy,vel_wz'
            )
            np.savetxt(str(csv_path), arr, delimiter=',', header=header, comments='', fmt='%.9f')
            saved.append(str(csv_path))

        self.get_logger().info(
            f'Saved {len(arr)} samples ({arr.shape}) to:\n  ' + '\n  '.join(saved)
        )

    def _shutdown_handler(self, signum, frame):
        self.get_logger().info('Shutdown signal received. Saving data...')
        self._save_data(suffix='_final')
        rclpy.try_shutdown()

    def save_and_shutdown(self):
        """Called on node destroy."""
        self._save_data(suffix='_final')


def main(args=None):
    rclpy.init(args=args)
    node = FrankaDataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt - saving and shutting down.')
    finally:
        node.save_and_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
