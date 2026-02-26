#!/usr/bin/env python3
"""
franka_data_inspector.py
========================
Offline tool to inspect, visualize and validate collected follower robot data.

Usage:
    python3 franka_data_inspector.py --file /tmp/franka_data/franka_follower_*.npy
    python3 franka_data_inspector.py --file /tmp/franka_data/franka_follower_*.csv
    python3 franka_data_inspector.py --dir /tmp/franka_data --merge
    python3 franka_data_inspector.py --file data.npy --plot
    python3 franka_data_inspector.py --file data.npy --export merged.npy
"""

import argparse
import sys
from pathlib import Path

import numpy as np

# ── Column definitions ──────────────────────────────────────────────────────

COLUMNS = {
    'timestamp_ns': 0,
    'q1': 1,  'q2': 2,  'q3': 3,  'q4': 4,  'q5': 5,  'q6': 6,  'q7': 7,
    'dq1': 8, 'dq2': 9, 'dq3': 10, 'dq4': 11, 'dq5': 12, 'dq6': 13, 'dq7': 14,
    'tau1': 15, 'tau2': 16, 'tau3': 17, 'tau4': 18, 'tau5': 19, 'tau6': 20, 'tau7': 21,
    'tau_ext1': 22, 'tau_ext2': 23, 'tau_ext3': 24, 'tau_ext4': 25,
    'tau_ext5': 26, 'tau_ext6': 27, 'tau_ext7': 28,
    'pos_x': 29, 'pos_y': 30, 'pos_z': 31,
    'quat_x': 32, 'quat_y': 33, 'quat_z': 34, 'quat_w': 35,
    'vel_vx': 36, 'vel_vy': 37, 'vel_vz': 38,
    'vel_wx': 39, 'vel_wy': 40, 'vel_wz': 41,
}

COL_NAMES = list(COLUMNS.keys())

GROUPS = {
    'joint_positions': list(range(1, 8)),
    'joint_velocities': list(range(8, 15)),
    'measured_torques': list(range(15, 22)),
    'external_torques': list(range(22, 29)),
    'cartesian_position': list(range(29, 32)),
    'quaternion': list(range(32, 36)),
    'cartesian_velocity': list(range(36, 42)),
}


def load_file(path: Path) -> np.ndarray:
    """Load CSV or NPY file."""
    path = Path(path)
    if path.suffix == '.npy':
        data = np.load(str(path))
        print(f'Loaded NPY: {path} → shape {data.shape}')
    elif path.suffix == '.csv':
        data = np.loadtxt(str(path), delimiter=',', skiprows=1)
        print(f'Loaded CSV: {path} → shape {data.shape}')
    else:
        raise ValueError(f'Unsupported file format: {path.suffix}')
    return data


def print_summary(data: np.ndarray):
    """Print a rich summary of the dataset."""
    n_samples = data.shape[0]
    n_cols = data.shape[1]

    ts = data[:, 0]
    duration_ns = ts[-1] - ts[0] if n_samples > 1 else 0
    duration_s = duration_ns * 1e-9

    dt = np.diff(ts)
    mean_dt_ms = np.mean(dt) * 1e-6 if len(dt) > 0 else 0
    actual_hz = 1e9 / np.mean(dt) if len(dt) > 0 else 0

    print('\n' + '=' * 65)
    print('  FRANKA FOLLOWER DATA SUMMARY')
    print('=' * 65)
    print(f'  Samples         : {n_samples:,}')
    print(f'  Columns         : {n_cols}  (expected 42)')
    print(f'  Duration        : {duration_s:.2f} s')
    print(f'  Mean dt         : {mean_dt_ms:.3f} ms  ({actual_hz:.1f} Hz)')
    print(f'  Timestamp range : {ts[0]:.0f} → {ts[-1]:.0f} ns')

    print('\n  PER-GROUP STATISTICS')
    print('  ' + '-' * 63)
    for group_name, col_indices in GROUPS.items():
        group_data = data[:, col_indices]
        col_labels = [COL_NAMES[i] for i in col_indices]
        print(f'\n  [{group_name}]')
        print(f'  {"Column":<12} {"Mean":>10} {"Std":>10} {"Min":>10} {"Max":>10}')
        print(f'  {"-"*52}')
        for i, col in enumerate(col_labels):
            d = group_data[:, i]
            print(f'  {col:<12} {np.mean(d):>10.4f} {np.std(d):>10.4f} '
                  f'{np.min(d):>10.4f} {np.max(d):>10.4f}')

    print('\n  DATA QUALITY CHECKS')
    print('  ' + '-' * 63)

    # Check for NaN/Inf
    nan_count = np.sum(np.isnan(data))
    inf_count = np.sum(np.isinf(data))
    print(f'  NaN values      : {nan_count}  {"✓" if nan_count == 0 else "⚠ WARNING"}')
    print(f'  Inf values      : {inf_count}  {"✓" if inf_count == 0 else "⚠ WARNING"}')

    # Check for zero external torques (means franka_robot_state not received)
    ext_torque_data = data[:, 22:29]
    zero_rows = np.sum(np.all(ext_torque_data == 0, axis=1))
    pct = 100 * zero_rows / n_samples
    print(f'  Zero ext torque : {zero_rows:,} rows ({pct:.1f}%)'
          f'  {"✓" if pct < 5 else "⚠ Check franka_robot_state topic"}')

    # Check for zero cartesian pos (means EE pose not received)
    cart_pos_data = data[:, 29:32]
    zero_cart = np.sum(np.all(cart_pos_data == 0, axis=1))
    pct_cart = 100 * zero_cart / n_samples
    print(f'  Zero cart pos   : {zero_cart:,} rows ({pct_cart:.1f}%)'
          f'  {"✓" if pct_cart < 5 else "⚠ Check franka_robot_state topic"}')

    print('=' * 65 + '\n')


def plot_data(data: np.ndarray):
    """Plot key signals."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib not available. Install with: pip3 install matplotlib')
        return

    ts = (data[:, 0] - data[0, 0]) * 1e-9  # seconds from start

    fig, axes = plt.subplots(4, 2, figsize=(16, 14))
    fig.suptitle('Franka Follower Robot Data', fontsize=14, fontweight='bold')

    # Joint positions
    ax = axes[0, 0]
    for i in range(7):
        ax.plot(ts, data[:, 1 + i], label=f'q{i+1}', linewidth=0.8)
    ax.set_title('Joint Positions (rad)')
    ax.set_xlabel('Time (s)')
    ax.legend(loc='upper right', fontsize=7, ncol=4)
    ax.grid(True, alpha=0.3)

    # Joint velocities
    ax = axes[0, 1]
    for i in range(7):
        ax.plot(ts, data[:, 8 + i], label=f'dq{i+1}', linewidth=0.8)
    ax.set_title('Joint Velocities (rad/s)')
    ax.set_xlabel('Time (s)')
    ax.legend(loc='upper right', fontsize=7, ncol=4)
    ax.grid(True, alpha=0.3)

    # Measured torques
    ax = axes[1, 0]
    for i in range(7):
        ax.plot(ts, data[:, 15 + i], label=f'τ{i+1}', linewidth=0.8)
    ax.set_title('Measured Torques (Nm)')
    ax.set_xlabel('Time (s)')
    ax.legend(loc='upper right', fontsize=7, ncol=4)
    ax.grid(True, alpha=0.3)

    # External torques
    ax = axes[1, 1]
    for i in range(7):
        ax.plot(ts, data[:, 22 + i], label=f'τ_ext{i+1}', linewidth=0.8)
    ax.set_title('External Torques (Nm)')
    ax.set_xlabel('Time (s)')
    ax.legend(loc='upper right', fontsize=7, ncol=4)
    ax.grid(True, alpha=0.3)

    # Cartesian position
    ax = axes[2, 0]
    for i, label in enumerate(['x', 'y', 'z']):
        ax.plot(ts, data[:, 29 + i], label=label, linewidth=0.8)
    ax.set_title('End-Effector Position (m)')
    ax.set_xlabel('Time (s)')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # Quaternion
    ax = axes[2, 1]
    for i, label in enumerate(['qx', 'qy', 'qz', 'qw']):
        ax.plot(ts, data[:, 32 + i], label=label, linewidth=0.8)
    ax.set_title('End-Effector Quaternion')
    ax.set_xlabel('Time (s)')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # Cartesian linear velocity
    ax = axes[3, 0]
    for i, label in enumerate(['vx', 'vy', 'vz']):
        ax.plot(ts, data[:, 36 + i], label=label, linewidth=0.8)
    ax.set_title('Cartesian Linear Velocity (m/s)')
    ax.set_xlabel('Time (s)')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # Cartesian angular velocity
    ax = axes[3, 1]
    for i, label in enumerate(['ωx', 'ωy', 'ωz']):
        ax.plot(ts, data[:, 39 + i], label=label, linewidth=0.8)
    ax.set_title('Cartesian Angular Velocity (rad/s)')
    ax.set_xlabel('Time (s)')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('/tmp/franka_data_plot.png', dpi=150, bbox_inches='tight')
    print('Plot saved to: /tmp/franka_data_plot.png')
    plt.show()


def merge_files(paths) -> np.ndarray:
    """Load and concatenate multiple files in chronological order."""
    arrays = []
    for p in sorted(paths):
        try:
            arrays.append(load_file(p))
        except Exception as e:
            print(f'  WARNING: skipping {p}: {e}')
    if not arrays:
        raise RuntimeError('No files loaded.')
    merged = np.vstack(arrays)
    # Sort by timestamp
    merged = merged[np.argsort(merged[:, 0])]
    print(f'\nMerged {len(arrays)} files → {merged.shape[0]:,} total samples')
    return merged


def main():
    parser = argparse.ArgumentParser(
        description='Inspect and analyze Franka follower data files.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('--file', nargs='+', help='One or more .npy or .csv files to load')
    parser.add_argument('--dir', help='Directory containing data files (loads all .npy or .csv)')
    parser.add_argument('--merge', action='store_true',
                        help='Merge multiple files (used with --dir or multiple --file)')
    parser.add_argument('--plot', action='store_true', help='Show plots')
    parser.add_argument('--export', help='Export merged/loaded data to this path (.npy or .csv)')
    parser.add_argument('--ext', default='npy', choices=['npy', 'csv'],
                        help='Extension to search for in --dir mode (default: npy)')

    args = parser.parse_args()

    # Collect file paths
    paths = []
    if args.file:
        paths = [Path(f) for f in args.file]
    elif args.dir:
        paths = sorted(Path(args.dir).glob(f'*.{args.ext}'))
        print(f'Found {len(paths)} .{args.ext} files in {args.dir}')
    else:
        parser.print_help()
        sys.exit(1)

    if not paths:
        print('No files found.')
        sys.exit(1)

    # Load data
    if len(paths) == 1 and not args.merge:
        data = load_file(paths[0])
    else:
        data = merge_files(paths)

    print_summary(data)

    if args.plot:
        plot_data(data)

    if args.export:
        export_path = Path(args.export)
        if export_path.suffix == '.npy':
            np.save(str(export_path), data)
            print(f'Exported {data.shape[0]:,} samples to: {export_path}')
        elif export_path.suffix == '.csv':
            header = ','.join(COL_NAMES)
            np.savetxt(str(export_path), data, delimiter=',', header=header,
                       comments='', fmt='%.9f')
            print(f'Exported {data.shape[0]:,} samples to: {export_path}')
        else:
            print(f'Unknown export format: {export_path.suffix}')


if __name__ == '__main__':
    main()
