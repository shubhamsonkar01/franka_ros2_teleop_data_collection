#!/usr/bin/env python3
"""
Franka Gripper Controller
=========================
Lets you OPEN and CLOSE the Franka gripper from the terminal.

Usage (run inside the Docker container):
  python3 gripper_controller.py                    # interactive keyboard mode
  python3 gripper_controller.py --open             # just open and exit
  python3 gripper_controller.py --close            # just close and exit
  python3 gripper_controller.py --width 0.05       # open to specific width (m)

Keyboard controls (interactive mode):
  o  → OPEN  gripper  (width = 0.08 m, full open)
  c  → CLOSE gripper  (grasp with default force = 20 N)
  g  → GRASP with custom width + force
  q  → Quit

How it works:
  Sends action goals to the Franka gripper action servers:
    /franka_gripper/move   (MoveAction)  — for open/move to width
    /franka_gripper/grasp  (GraspAction) — for close/grasp with force

  These action servers are started automatically by franka_ros2
  when the gripper is enabled (load_gripper:=true in your bringup).

Gripper topic for monitoring:
  /franka_gripper/joint_states  — finger positions, velocity, effort

Notes:
  - Width is TOTAL gap between fingers (0 = fully closed, 0.08 = fully open)
  - Each finger moves half the total width
  - Speed default: 0.1 m/s (safe for most tasks)
  - Grasp force default: 20 N (adjust for your task)
  - epsilon (grasp tolerance): inner=0.005, outer=0.005 m
"""

import argparse
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

try:
    from franka_msgs.action import Move, Grasp
    FRANKA_MSGS_OK = True
except ImportError:
    FRANKA_MSGS_OK = False
    print('[ERROR] franka_msgs not found. Make sure franka_ros2 is built and sourced.')
    print('        Inside Docker: source /ros2_ws/install/setup.bash')

from sensor_msgs.msg import JointState


# ── Constants ─────────────────────────────────────────────────────────────────

GRIPPER_OPEN_WIDTH  = 0.08   # m — full open (Franka Hand max aperture)
GRIPPER_SPEED       = 0.1    # m/s
GRASP_FORCE         = 20.0   # N
GRASP_EPSILON_INNER = 0.005  # m
GRASP_EPSILON_OUTER = 0.005  # m


# ── Node ──────────────────────────────────────────────────────────────────────

class GripperController(Node):

    def __init__(self, gripper_namespace='franka_gripper'):
        super().__init__('gripper_controller')

        ns = gripper_namespace.rstrip('/')

        if FRANKA_MSGS_OK:
            self._move_client  = ActionClient(self, Move,  f'/{ns}/move')
            self._grasp_client = ActionClient(self, Grasp, f'/{ns}/grasp')
        else:
            self._move_client  = None
            self._grasp_client = None

        # Monitor gripper state
        self._left_pos  = 0.0
        self._right_pos = 0.0
        self.create_subscription(
            JointState,
            f'/{ns}/joint_states',
            self._state_cb,
            10,
        )

        self.get_logger().info(
            f'GripperController ready\n'
            f'  Move  action: /{ns}/move\n'
            f'  Grasp action: /{ns}/grasp\n'
            f'  State topic:  /{ns}/joint_states\n'
        )

    def _state_cb(self, msg):
        if len(msg.position) >= 2:
            self._left_pos  = msg.position[0]
            self._right_pos = msg.position[1]

    @property
    def current_width(self):
        return self._left_pos + self._right_pos

    def open(self, width=GRIPPER_OPEN_WIDTH, speed=GRIPPER_SPEED):
        """Move gripper to width (m). Default = fully open."""
        if not FRANKA_MSGS_OK or self._move_client is None:
            self.get_logger().error('franka_msgs not available')
            return False

        self.get_logger().info(f'Opening gripper → width={width:.4f} m, speed={speed:.3f} m/s')

        if not self._move_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Move action server not available!')
            return False

        goal = Move.Goal()
        goal.width = float(width)
        goal.speed = float(speed)

        future = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Move goal rejected!')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)
        self.get_logger().info(f'Gripper open done. Current width ≈ {self.current_width:.4f} m')
        return True

    def close(self, width=0.0, speed=GRIPPER_SPEED,
              force=GRASP_FORCE,
              epsilon_inner=GRASP_EPSILON_INNER,
              epsilon_outer=GRASP_EPSILON_OUTER):
        """
        Grasp at target width with force.
        width=0.0 means close fully (use for grasping unknown objects too — 
        set width ≈ object diameter, gripper will stop when it touches).
        """
        if not FRANKA_MSGS_OK or self._grasp_client is None:
            self.get_logger().error('franka_msgs not available')
            return False

        self.get_logger().info(
            f'Closing gripper → width={width:.4f} m, '
            f'force={force:.1f} N, speed={speed:.3f} m/s'
        )

        if not self._grasp_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Grasp action server not available!')
            return False

        goal = Grasp.Goal()
        goal.width          = float(width)
        goal.speed          = float(speed)
        goal.force          = float(force)
        goal.epsilon.inner  = float(epsilon_inner)
        goal.epsilon.outer  = float(epsilon_outer)

        future = self._grasp_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Grasp goal rejected!')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)
        self.get_logger().info(f'Gripper close done. Current width ≈ {self.current_width:.4f} m')
        return True


# ── Interactive keyboard loop ─────────────────────────────────────────────────

def keyboard_loop(gripper: GripperController):
    print('\n' + '='*50)
    print('  Franka Gripper Controller — Keyboard Mode')
    print('='*50)
    print('  o        → OPEN  (full open, 0.08 m)')
    print('  c        → CLOSE (grasp, 20 N force)')
    print('  w <val>  → open to specific WIDTH in meters')
    print('             e.g.  w 0.05  → open to 5 cm')
    print('  f <val>  → set grasp FORCE in N (default 20)')
    print('             e.g.  f 40')
    print('  q        → quit')
    print('='*50)

    grasp_force = GRASP_FORCE

    while True:
        try:
            cmd = input('\nCommand: ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if cmd == 'q':
            break
        elif cmd == 'o':
            gripper.open()
        elif cmd == 'c':
            gripper.close(force=grasp_force)
        elif cmd.startswith('w '):
            try:
                w = float(cmd.split()[1])
                if 0.0 <= w <= 0.08:
                    gripper.open(width=w)
                else:
                    print('  Width must be 0.0 – 0.08 m')
            except (IndexError, ValueError):
                print('  Usage: w <width_in_meters>  e.g. w 0.05')
        elif cmd.startswith('f '):
            try:
                grasp_force = float(cmd.split()[1])
                print(f'  Grasp force set to {grasp_force:.1f} N')
            except (IndexError, ValueError):
                print('  Usage: f <force_in_N>  e.g. f 40')
        else:
            print('  Unknown command. Use o / c / w <val> / f <val> / q')

    print('Exiting gripper controller.')


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Franka Gripper Controller')
    parser.add_argument('--namespace', default='franka_gripper',
                        help='Gripper action server namespace (default: franka_gripper)')
    parser.add_argument('--open',   action='store_true', help='Open gripper and exit')
    parser.add_argument('--close',  action='store_true', help='Close gripper and exit')
    parser.add_argument('--width',  type=float, default=None,
                        help='Open to specific width in meters (0.0–0.08)')
    parser.add_argument('--force',  type=float, default=GRASP_FORCE,
                        help=f'Grasp force in N (default: {GRASP_FORCE})')
    args = parser.parse_args()

    rclpy.init()
    node = GripperController(gripper_namespace=args.namespace)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        if args.open:
            w = args.width if args.width is not None else GRIPPER_OPEN_WIDTH
            node.open(width=w)
        elif args.close:
            node.close(force=args.force)
        elif args.width is not None:
            node.open(width=args.width)
        else:
            # Interactive keyboard mode
            keyboard_loop(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
