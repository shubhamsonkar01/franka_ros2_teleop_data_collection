#!/usr/bin/env python3
"""
Franka Data Collection Node — FIXED + GRIPPER SUPPORT
======================================================

FIXES vs original:
  1. vel_vx/vy/vz/wx/wy/wz were always 0 during kinesthetic teaching.
     ROOT CAUSE: o_dp_ee_d / o_dp_ee_c = DESIRED velocity, always 0 during
     gravity comp. We now compute velocity via finite diff of O_T_EE position.
  2. Added gripper state (finger positions + velocity + effort).

NO controller change needed. Stay in gravity compensation / kinesthetic mode.
The robot's actual motion is captured via position finite-differencing.

Data format per row — NOW 46 values (was 42):
  [0]      timestamp_ns
  [1-7]    joint positions       q1..q7
  [8-14]   joint velocities      dq1..dq7
  [15-21]  measured torques      tau1..tau7
  [22-28]  external torques      tau_ext1..tau_ext7
  [29-31]  EE Cartesian position pos_x, pos_y, pos_z
  [32-35]  EE quaternion         quat_x, quat_y, quat_z, quat_w
  [36-41]  EE Cartesian velocity vel_vx, vel_vy, vel_vz, vel_wx, vel_wy, vel_wz  ← FIXED
  [42]     gripper_left_pos      left finger position  (m)
  [43]     gripper_right_pos     right finger position (m)
  [44]     gripper_vel           finger velocity       (m/s, mean of both)
  [45]     gripper_effort        finger effort/force   (N, mean of both)
"""

import signal
import threading
from pathlib import Path
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState

try:
    from franka_msgs.msg import FrankaRobotState
    FRANKA_STATE_AVAILABLE = True
except ImportError:
    FRANKA_STATE_AVAILABLE = False

NUM_JOINTS  = 7
STATE_DIM   = 46   # 42 original + 4 gripper


# ── Helpers ───────────────────────────────────────────────────────────────────

def rotation_matrix_to_quaternion(R):
    """3x3 rotation matrix → quaternion [qx, qy, qz, qw]."""
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


def quat_to_angular_velocity(q_prev, q_curr, dt):
    """
    Compute angular velocity [wx, wy, wz] from two unit quaternions [qx,qy,qz,qw].
    Uses the relation: omega = 2 * E(q)^T * dq/dt
    """
    if dt < 1e-9:
        return np.zeros(3)
    q_dot = (q_curr - q_prev) / dt
    qx, qy, qz, qw = q_curr
    # E matrix maps q_dot → angular velocity in world frame
    E = np.array([
        [-qx,  qw,  qz, -qy],
        [-qy, -qz,  qw,  qx],
        [-qz,  qy, -qx,  qw],
    ])
    return 2.0 * (E @ q_dot)


# ── Main Node ─────────────────────────────────────────────────────────────────

class FrankaDataCollector(Node):

    def __init__(self):
        super().__init__('franka_data_collector')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('follower_namespace',      'franka_teleop/follower')
        self.declare_parameter('output_dir',              '/tmp/franka_data')
        self.declare_parameter('save_csv',                True)
        self.declare_parameter('save_npy',                True)
        self.declare_parameter('buffer_size',             100000)
        self.declare_parameter('auto_save_interval_sec',  30.0)
        self.declare_parameter('use_franka_robot_state',  True)
        self.declare_parameter('vel_ema_alpha',           0.5)   # 0=raw 1=frozen

        self.follower_ns       = self.get_parameter('follower_namespace').value
        self.output_dir        = Path(self.get_parameter('output_dir').value)
        self.save_csv          = self.get_parameter('save_csv').value
        self.save_npy          = self.get_parameter('save_npy').value
        self.buffer_size       = self.get_parameter('buffer_size').value
        self.auto_save_interval= self.get_parameter('auto_save_interval_sec').value
        self.use_franka_state  = self.get_parameter('use_franka_robot_state').value
        self.vel_ema_alpha     = self.get_parameter('vel_ema_alpha').value

        self.output_dir.mkdir(parents=True, exist_ok=True)

        # ── Internal state ───────────────────────────────────────────────────
        self.lock = threading.Lock()
        self.data_buffer = []

        # Robot arm state
        self._joint_pos       = np.zeros(NUM_JOINTS)
        self._joint_vel       = np.zeros(NUM_JOINTS)
        self._measured_torques= np.zeros(NUM_JOINTS)
        self._ext_torques     = np.zeros(NUM_JOINTS)
        self._cart_pos        = np.zeros(3)
        self._quat            = np.array([0.0, 0.0, 0.0, 1.0])  # xyzw
        self._cart_vel        = np.zeros(6)  # computed via finite diff

        # Gripper state — from /franka_gripper/joint_states
        # Two fingers: panda_finger_joint1 (left), panda_finger_joint2 (right)
        self._gripper_left_pos  = 0.0   # m
        self._gripper_right_pos = 0.0   # m
        self._gripper_vel       = 0.0   # m/s (mean)
        self._gripper_effort    = 0.0   # N   (mean)
        self._gripper_received  = False

        # Finite-diff velocity state
        self._prev_cart_pos = None
        self._prev_quat     = None
        self._prev_ts_ns    = None
        self._vel_ema       = np.zeros(6)

        self._session_start = datetime.now().strftime('%Y%m%d_%H%M%S')

        # ── QoS ─────────────────────────────────────────────────────────────
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Subscriptions ────────────────────────────────────────────────────
        ns = self.follower_ns.rstrip('/')

        # Joint states (q, dq, tau)
        self.create_subscription(
            JointState,
            f'/{ns}/franka_robot_state_broadcaster/measured_joint_states',
            self._joint_state_callback,
            best_effort_qos,
        )

        # FrankaRobotState (O_T_EE for position+orientation, ext torques)
        if self.use_franka_state and FRANKA_STATE_AVAILABLE:
            self.create_subscription(
                FrankaRobotState,
                f'/{ns}/franka_robot_state_broadcaster/robot_state',
                self._franka_state_callback,
                best_effort_qos,
            )
        else:
            if not FRANKA_STATE_AVAILABLE:
                self.get_logger().warn('franka_msgs not found — EE pose will be zeros.')

        # External torques
        self.create_subscription(
            JointState,
            f'/{ns}/franka_robot_state_broadcaster/external_joint_torques',
            self._ext_torque_callback,
            best_effort_qos,
        )

        # ── GRIPPER: /franka_gripper/joint_states ───────────────────────────
        # Topic: /franka_teleop/follower/franka_gripper/joint_states
        # OR:    /franka_gripper/joint_states  (if gripper runs at top level)
        # We try both — the one that exists will fire callbacks.
        gripper_topic_ns   = f'/{ns}/franka_gripper/joint_states'
        gripper_topic_root = '/franka_gripper/joint_states'

        self.create_subscription(
            JointState, gripper_topic_ns,
            self._gripper_callback, best_effort_qos,
        )
        self.create_subscription(
            JointState, gripper_topic_root,
            self._gripper_callback, best_effort_qos,
        )
        self.get_logger().info(
            f'Gripper topics tried:\n'
            f'  {gripper_topic_ns}\n'
            f'  {gripper_topic_root}'
        )

        # ── Timers ──────────────────────────────────────────────────────────
        self.create_timer(0.001, self._record_callback)   # 1 kHz recording
        if self.auto_save_interval > 0:
            self.create_timer(self.auto_save_interval, self._auto_save)

        self.get_logger().info(
            f'\nFrankaDataCollector started  (STATE_DIM={STATE_DIM})\n'
            f'  Namespace   : {self.follower_ns}\n'
            f'  Output dir  : {self.output_dir}\n'
            f'  EMA alpha   : {self.vel_ema_alpha}\n'
            f'  Velocity    : computed from finite diff of O_T_EE (not o_dp_ee_d)\n'
            f'  Gripper     : cols 42-45  [left_pos, right_pos, vel, effort]\n'
        )

        signal.signal(signal.SIGINT,  self._shutdown_handler)
        signal.signal(signal.SIGTERM, self._shutdown_handler)

    # ── Topic callbacks ───────────────────────────────────────────────────────

    def _joint_state_callback(self, msg: JointState):
        n = min(len(msg.position), NUM_JOINTS)
        with self.lock:
            if n > 0:
                self._joint_pos[:n] = msg.position[:n]
            nv = min(len(msg.velocity), NUM_JOINTS)
            if nv > 0:
                self._joint_vel[:nv] = msg.velocity[:nv]
            ne = min(len(msg.effort), NUM_JOINTS)
            if ne > 0:
                self._measured_torques[:ne] = msg.effort[:ne]

    def _ext_torque_callback(self, msg: JointState):
        n = min(len(msg.effort), NUM_JOINTS)
        with self.lock:
            if n > 0:
                self._ext_torques[:n] = msg.effort[:n]

    def _franka_state_callback(self, msg):
        """
        Read O_T_EE for position + orientation.
        We do NOT read o_dp_ee_d/o_dp_ee_c — always 0 in gravity comp mode.
        Velocity is computed in _record_callback via finite differences.
        """
        with self.lock:
            if hasattr(msg, 'o_t_ee') and len(msg.o_t_ee) == 16:
                T = np.array(msg.o_t_ee).reshape(4, 4, order='F')
                self._cart_pos = T[:3, 3].copy()
                self._quat = rotation_matrix_to_quaternion(T[:3, :3])
            elif hasattr(msg, 'o_t_ee_d') and len(msg.o_t_ee_d) == 16:
                T = np.array(msg.o_t_ee_d).reshape(4, 4, order='F')
                self._cart_pos = T[:3, 3].copy()
                self._quat = rotation_matrix_to_quaternion(T[:3, :3])

            if hasattr(msg, 'tau_ext_hat_filtered') and \
               len(msg.tau_ext_hat_filtered) == NUM_JOINTS:
                self._ext_torques = np.array(msg.tau_ext_hat_filtered)

    def _gripper_callback(self, msg: JointState):
        """
        Franka gripper publishes a JointState with 2 entries:
          name:     ['panda_finger_joint1', 'panda_finger_joint2']
          position: [left_pos_m,  right_pos_m]   typically 0..0.04 each (total 0..0.08m)
          velocity: [left_vel,    right_vel]
          effort:   [left_force,  right_force]

        Gripper OPEN:  position ~0.04 each (total width ~8cm)
        Gripper CLOSE: position ~0.0  each
        """
        with self.lock:
            n = len(msg.position)
            if n >= 1:
                self._gripper_left_pos  = msg.position[0]
            if n >= 2:
                self._gripper_right_pos = msg.position[1]

            nv = len(msg.velocity)
            if nv >= 2:
                self._gripper_vel = (msg.velocity[0] + msg.velocity[1]) / 2.0
            elif nv == 1:
                self._gripper_vel = msg.velocity[0]

            ne = len(msg.effort)
            if ne >= 2:
                self._gripper_effort = (msg.effort[0] + msg.effort[1]) / 2.0
            elif ne == 1:
                self._gripper_effort = msg.effort[0]

            self._gripper_received = True

    # ── Recording ─────────────────────────────────────────────────────────────

    def _record_callback(self):
        ts_ns = self.get_clock().now().nanoseconds

        with self.lock:
            curr_pos  = self._cart_pos.copy()
            curr_quat = self._quat.copy()

            # ── Finite-diff velocity ────────────────────────────────────────
            if self._prev_cart_pos is not None:
                dt = (ts_ns - self._prev_ts_ns) * 1e-9
                if dt > 1e-6:
                    lin_vel = (curr_pos - self._prev_cart_pos) / dt
                    ang_vel = quat_to_angular_velocity(self._prev_quat, curr_quat, dt)
                    raw_vel = np.clip(
                        np.concatenate([lin_vel, ang_vel]), -5.0, 5.0
                    )
                    a = self.vel_ema_alpha
                    self._vel_ema  = a * self._vel_ema + (1.0 - a) * raw_vel
                    self._cart_vel = self._vel_ema.copy()

            self._prev_cart_pos = curr_pos.copy()
            self._prev_quat     = curr_quat.copy()
            self._prev_ts_ns    = ts_ns
            # ───────────────────────────────────────────────────────────────

            # Assemble 46-dim row
            row = np.empty(STATE_DIM)
            row[0]     = float(ts_ns)
            row[1:8]   = self._joint_pos
            row[8:15]  = self._joint_vel
            row[15:22] = self._measured_torques
            row[22:29] = self._ext_torques
            row[29:32] = curr_pos
            row[32:36] = curr_quat              # qx qy qz qw
            row[36:42] = self._cart_vel         # FIXED: non-zero during teaching
            row[42]    = self._gripper_left_pos
            row[43]    = self._gripper_right_pos
            row[44]    = self._gripper_vel
            row[45]    = self._gripper_effort

            self.data_buffer.append(row)

        if len(self.data_buffer) >= self.buffer_size:
            self._save_data(suffix='_partial')

    # ── Save ──────────────────────────────────────────────────────────────────

    def _auto_save(self):
        n = len(self.data_buffer)
        if n > 0:
            self.get_logger().info(f'Auto-save: {n} samples so far.')
            self._save_data(suffix='_autosave')

    def _save_data(self, suffix=''):
        with self.lock:
            if not self.data_buffer:
                return
            arr = np.array(self.data_buffer, dtype=np.float64)
            self.data_buffer = []

        ts   = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        name = f'franka_follower_{self._session_start}{suffix}_{ts}'

        saved = []
        if self.save_npy:
            p = self.output_dir / f'{name}.npy'
            np.save(str(p), arr)
            saved.append(str(p))

        if self.save_csv:
            p = self.output_dir / f'{name}.csv'
            header = (
                'timestamp_ns,'
                'q1,q2,q3,q4,q5,q6,q7,'
                'dq1,dq2,dq3,dq4,dq5,dq6,dq7,'
                'tau1,tau2,tau3,tau4,tau5,tau6,tau7,'
                'tau_ext1,tau_ext2,tau_ext3,tau_ext4,tau_ext5,tau_ext6,tau_ext7,'
                'pos_x,pos_y,pos_z,'
                'quat_x,quat_y,quat_z,quat_w,'
                'vel_vx,vel_vy,vel_vz,vel_wx,vel_wy,vel_wz,'
                'gripper_left_pos,gripper_right_pos,gripper_vel,gripper_effort'
            )
            np.savetxt(str(p), arr, delimiter=',',
                       header=header, comments='', fmt='%.9f')
            saved.append(str(p))

        # Sanity-check printout on save
        v = arr[:, 36:42]
        g = arr[:, 42:46]
        gripper_rcvd = self._gripper_received
        self.get_logger().info(
            f'Saved {len(arr)} rows → ' + ', '.join(saved) + '\n'
            f'  vel_vx  [{v[:,0].min():.3f}, {v[:,0].max():.3f}]  '
            f'vy [{v[:,1].min():.3f}, {v[:,1].max():.3f}]  '
            f'vz [{v[:,2].min():.3f}, {v[:,2].max():.3f}]\n'
            f'  gripper_left  [{g[:,0].min():.4f}, {g[:,0].max():.4f}] m  '
            f'right [{g[:,1].min():.4f}, {g[:,1].max():.4f}] m  '
            f'(topic received: {gripper_rcvd})'
        )

    def _shutdown_handler(self, signum, frame):
        self.get_logger().info('Shutdown — saving final data...')
        self._save_data(suffix='_final')
        rclpy.try_shutdown()

    def save_and_shutdown(self):
        self._save_data(suffix='_final')


def main(args=None):
    rclpy.init(args=args)
    node = FrankaDataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_and_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
