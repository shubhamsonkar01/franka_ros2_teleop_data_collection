<div align="center">

# 🤖 franka_ros2_teleop
### Franka FR3 Teleoperation + Kinesthetic Teaching + Data Collection

[![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros)](https://docs.ros.org/en/humble/)
[![Docker](https://img.shields.io/badge/Docker-Ready-2496ED?logo=docker)](https://www.docker.com/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green)](LICENSE)
[![Franka](https://img.shields.io/badge/Robot-Franka%20FR3-orange)](https://www.franka.de/)

*Leader-follower teleoperation for Franka FR3 robots with real-time state recording for robot learning.*

</div>

---

## 📋 Table of Contents

- [What This Repository Does](#-what-this-repository-does)
- [Repository Structure](#-repository-structure)
- [Where to Place the New Files](#-where-to-place-the-new-files)
- [Hardware Setup](#-hardware-setup)
- [Quick Start](#-quick-start)
- [Data Collection](#-data-collection)
- [Gripper Control](#-gripper-control)
- [Recorded Data Format](#-recorded-data-format)
- [Loading Data in Python](#-loading-data-in-python)
- [Configuration Reference](#-configuration-reference)
- [Troubleshooting](#-troubleshooting)

---

## 🎯 What This Repository Does

This repository lets you:

1. **Teleoperate** a Franka FR3 follower robot by physically moving a leader FR3 robot
2. **Record** every state of the follower robot at ~1 kHz during teaching — joint angles, torques, end-effector pose, **end-effector velocity**, and **gripper state**
3. **Control the gripper** interactively during recording for pick-and-place tasks
4. **Save** everything as CSV and NumPy files ready for robot learning pipelines

```
You move leader robot  →  Follower mirrors motion  →  All state recorded to disk
        🤖     ──────────────────────────────────────────→     🤖
      (leader)              teleoperation                   (follower)
                                                               ↓
                                                        franka_data/*.npy
                                                        franka_data/*.csv
```

> **Key fix in this version:** End-effector velocity (`vel_vx`, `vel_vy`, `vel_vz`) was previously always zero during kinesthetic teaching because the original code read the *desired* velocity from the controller, which is zero when the robot is in gravity compensation mode. This version computes velocity from **finite differences of the actual EE position** — no controller change needed.

---

## 📁 Repository Structure

```
franka_ros2_teleop/                         ← repo root
│
├── 📂 franka_data_collector/               ← ROS 2 data collection package
│   ├── franka_data_collector/
│   │   ├── data_collector_node.py          ← ⭐ MAIN collector (fixed + gripper)
│   │   └── __init__.py
│   ├── launch/
│   │   └── data_collector.launch.py        ← launch file for collector
│   ├── config/
│   │   └── data_collector_params.yaml      ← parameters config
│   ├── CMakeLists.txt
│   └── package.xml
│
├── 📂 scripts/                             ← ⭐ Utility scripts (NEW)
│   └── gripper_controller.py              ← interactive gripper open/close
│
├── 📂 franka_data/                         ← recorded data saved here
│   └── *.npy / *.csv
│
├── 📂 src/                                 ← C++ teleop controllers
│   ├── teleop_leader_controller.cpp
│   ├── teleop_follower_controller.cpp
│   ├── teleop_coordinator.cpp
│   └── teleop_gripper_node.cpp
│
├── 📂 include/franka_ros2_teleop/          ← C++ headers
│   ├── teleop_leader_controller.hpp
│   ├── teleop_follower_controller.hpp
│   └── utils.hpp
│
├── 📂 launch/                              ← ROS 2 launch files for teleop
│   ├── teleop.launch.py
│   └── teleop_single_robot.launch.py
│
├── 📂 config/                              ← Robot pair configuration
│   ├── fr3_teleop_config.yaml             ← single pair setup
│   ├── fr3_duo_teleop_config.yaml         ← dual arm setup
│   └── teleop_controllers.yaml
│
├── franka_data_inspector.py               ← inspect/plot recorded data
├── Dockerfile                             ← base teleop Docker image
├── Dockerfile.updated                     ← extended image with collector
├── docker-compose.yml                     ← teleop only
├── docker-compose.with_collector.yml      ← teleop + data collection
└── README.md
```

---

## 📂 Where to Place the New Files

You downloaded two new files. Here is **exactly** where they go:

```
franka_ros2_teleop/
│
├── franka_data_collector/
│   └── franka_data_collector/
│       └── data_collector_node.py    ← 🔴 REPLACE this file
│
└── scripts/
    └── gripper_controller.py         ← 🟢 CREATE this folder + file
```

**Step-by-step commands:**

```bash
# 1. Replace the data collector node
cp data_collector_node.py \
   franka_ros2_teleop/franka_data_collector/franka_data_collector/data_collector_node.py

# 2. Create the scripts folder and place the gripper controller
mkdir -p franka_ros2_teleop/scripts
cp gripper_controller.py franka_ros2_teleop/scripts/gripper_controller.py
```

---

## 🔌 Hardware Setup

```
┌─────────────────────────────────────────────────────────────┐
│                        Your Network                          │
│                                                             │
│    ┌──────────────┐              ┌──────────────┐           │
│    │  Leader FR3  │              │ Follower FR3 │           │
│    │ 192.168.1.12 │              │ 192.168.1.15 │           │
│    └──────┬───────┘              └──────┬───────┘           │
│           │                            │                    │
│           └──────────────┬─────────────┘                    │
│                          │                                  │
│                 ┌────────┴────────┐                         │
│                 │   Your PC       │                         │
│                 │  (Docker host)  │                         │
│                 └─────────────────┘                         │
└─────────────────────────────────────────────────────────────┘
```

**Before starting, confirm:**
- Both robots are powered on and showing a **blue light** (FCI enabled)
- Both robot IPs are reachable: `ping 192.168.1.12` and `ping 192.168.1.15`
- Franka Desk shows no active errors (open `192.168.1.15` in browser)
- Both robots are in **gravity compensation mode** (white light + joints can be moved freely)

---

## 🚀 Quick Start

### Step 1 — Clone the repository

```bash
git clone https://github.com/YOUR_USERNAME/franka_ros2_teleop.git
cd franka_ros2_teleop
```

### Step 2 — Configure your robot IPs

Edit `docker-compose.with_collector.yml` (or `docker-compose.yml`):

```yaml
x-teleop-config:
  pairs:
    - namespace: franka_teleop
      leader:
        robot_ip: 192.168.1.12    # ← your leader robot IP
      follower:
        robot_ip: 192.168.1.15    # ← your follower robot IP
```

### Step 3 — Create the data output directory

```bash
mkdir -p ./franka_data
```

### Step 4 — Build and launch everything

```bash
# Teleop + data collection together (recommended)
docker compose -f docker-compose.with_collector.yml up --build
```

Or if you only want teleoperation without recording:

```bash
docker compose up --build
```

### Step 5 — Start teaching!

Once you see `[franka_data_collector] FrankaDataCollector started` in the logs, the robot is ready. Move the **leader** robot by hand — the follower mirrors your motion and all state is being recorded.

### Step 6 — Stop and collect your data

Press `Ctrl+C` in the terminal. Data saves automatically to `./franka_data/`.

---

## 📊 Data Collection

### What gets recorded

At every ~1ms tick (≈1 kHz), the collector saves **46 values** per row:

| Columns | Field | Description | Unit |
|---------|-------|-------------|------|
| `[0]` | `timestamp_ns` | ROS clock timestamp | ns |
| `[1–7]` | `q1`…`q7` | Joint positions | rad |
| `[8–14]` | `dq1`…`dq7` | Joint velocities | rad/s |
| `[15–21]` | `tau1`…`tau7` | Measured joint torques | Nm |
| `[22–28]` | `tau_ext1`…`tau_ext7` | External joint torques | Nm |
| `[29–31]` | `pos_x`, `pos_y`, `pos_z` | EE Cartesian position | m |
| `[32–35]` | `quat_x`, `quat_y`, `quat_z`, `quat_w` | EE orientation | – |
| `[36–41]` | `vel_vx`, `vel_vy`, `vel_vz`, `vel_wx`, `vel_wy`, `vel_wz` | EE Cartesian velocity ✅ | m/s, rad/s |
| `[42]` | `gripper_left_pos` | Left finger position | m |
| `[43]` | `gripper_right_pos` | Right finger position | m |
| `[44]` | `gripper_vel` | Finger velocity (mean) | m/s |
| `[45]` | `gripper_effort` | Finger force (mean) | N |

> ✅ **Velocity fix:** `vel_vx/vy/vz` are now computed from finite differences of the actual EE position, not from the controller's desired velocity (which is always 0 in gravity comp mode).

### Output files

Files are saved to `./franka_data/` on your host with names like:

```
franka_follower_20260223_162545_final_20260223_162556.npy   ← main save on Ctrl+C
franka_follower_20260223_162545_autosave_20260223_162530.npy ← periodic auto-saves
franka_follower_20260223_162545_partial_20260223_162515.npy  ← buffer-full flushes
```

### Inspecting recorded data

No ROS needed — run this anywhere:

```bash
# Summary of a single file
python3 franka_data_inspector.py --file franka_data/franka_follower_final.npy

# Merge all files in a session + plot
python3 franka_data_inspector.py --dir franka_data/ --merge --plot

# Export merged data to a single NPY
python3 franka_data_inspector.py --dir franka_data/ --merge --export merged.npy

# Convert NPY to CSV
python3 franka_data_inspector.py --file data.npy --export data.csv
```

---

## 🤏 Gripper Control

The gripper controller lets you **open and close the gripper** during a recording session, enabling pick-and-place demonstrations.

### Run it inside the Docker container

Open a **second terminal** while the main teleop is running:

```bash
# Get a shell inside the running container
docker exec -it franka_ros2_teleop-franka_data_collector-1 bash

# Inside the container, run the gripper controller
python3 /ros2_ws/src/franka_ros2_teleop/scripts/gripper_controller.py
```

### Keyboard controls (interactive mode)

```
==========================================
  Franka Gripper Controller
==========================================
  o          →  OPEN  (full open, 8 cm)
  c          →  CLOSE (grasp, 20 N force)
  w <value>  →  open to specific width
               e.g.  w 0.05  →  5 cm gap
  f <value>  →  set grasp force in N
               e.g.  f 40    →  40 N
  q          →  quit
==========================================
```

### One-shot commands (no interactive mode)

```bash
# Open fully
python3 scripts/gripper_controller.py --open

# Close (default 20 N force)
python3 scripts/gripper_controller.py --close

# Open to specific width (e.g. 5 cm)
python3 scripts/gripper_controller.py --width 0.05

# Close with custom force
python3 scripts/gripper_controller.py --close --force 40
```

### Understanding gripper state in recorded data

```
Gripper OPEN:   gripper_left_pos + gripper_right_pos ≈ 0.08 m  (8 cm total)
Gripper CLOSED: gripper_left_pos + gripper_right_pos ≈ 0.00 m

For pick-and-place detection:
  total_width = data[:, 42] + data[:, 43]
  grasping     = total_width < 0.02   # gripping an object
  open         = total_width > 0.07   # hand open
```

---

## 🐍 Loading Data in Python

```python
import numpy as np

# ── Load ──────────────────────────────────────────────────────────────────
data = np.load('franka_data/franka_follower_final.npy')
# Shape: (N_samples, 46)

# ── Unpack columns ────────────────────────────────────────────────────────
timestamp    = data[:, 0]           # ns
joint_pos    = data[:, 1:8]         # rad   shape (N, 7)
joint_vel    = data[:, 8:15]        # rad/s shape (N, 7)
torque       = data[:, 15:22]       # Nm    shape (N, 7)
torque_ext   = data[:, 22:29]       # Nm    shape (N, 7)
ee_pos       = data[:, 29:32]       # m     shape (N, 3)  [x, y, z]
ee_quat      = data[:, 32:36]       # –     shape (N, 4)  [qx, qy, qz, qw]
ee_vel       = data[:, 36:42]       # m/s   shape (N, 6)  [vx,vy,vz,wx,wy,wz]
gripper_l    = data[:, 42]          # m     left finger position
gripper_r    = data[:, 43]          # m     right finger position
gripper_vel  = data[:, 44]          # m/s   finger velocity
gripper_f    = data[:, 45]          # N     finger force

# ── Time axis ─────────────────────────────────────────────────────────────
t = (timestamp - timestamp[0]) * 1e-9   # seconds from start

# ── Gripper events ────────────────────────────────────────────────────────
total_width  = gripper_l + gripper_r
is_grasping  = total_width < 0.02       # boolean array

print(f"Duration:   {t[-1]:.2f} s")
print(f"Samples:    {len(data)}")
print(f"Frequency:  {len(data)/t[-1]:.0f} Hz")
print(f"EE pos range X: [{ee_pos[:,0].min():.3f}, {ee_pos[:,0].max():.3f}] m")
print(f"EE vel range vx: [{ee_vel[:,0].min():.3f}, {ee_vel[:,0].max():.3f}] m/s")
print(f"Grasp events: {is_grasping.sum()} samples")
```

---

## ⚙️ Configuration Reference

### Robot IPs — `docker-compose.with_collector.yml`

```yaml
x-teleop-config:
  pairs:
    - namespace: franka_teleop
      leader:
        robot_ip: 192.168.1.12     # ← leader robot
      follower:
        robot_ip: 192.168.1.15     # ← follower robot
```

### Data collector params — `franka_data_collector/config/data_collector_params.yaml`

```yaml
franka_data_collector:
  ros__parameters:
    follower_namespace: "franka_teleop/follower"  # must match your pair namespace + /follower
    output_dir: "/tmp/franka_data"                # inside container → mounted to ./franka_data on host
    save_csv: true
    save_npy: true
    buffer_size: 100000                           # ~100 seconds at 1 kHz before auto-flush
    auto_save_interval_sec: 30.0                  # periodic save every 30s
    use_franka_robot_state: true                  # requires franka_msgs
    vel_ema_alpha: 0.5                            # velocity smoothing (0=raw, 1=frozen)
```

### Launch arguments (override at runtime)

```bash
ros2 launch franka_data_collector data_collector.launch.py \
    follower_namespace:=franka_teleop/follower \
    output_dir:=/tmp/franka_data \
    save_csv:=true \
    save_npy:=true \
    auto_save_interval_sec:=30.0
```

### Finding your `follower_namespace`

```bash
# While the teleop is running, check what topics exist:
ros2 topic list | grep robot_state

# You will see something like:
# /franka_teleop/follower/franka_robot_state_broadcaster/robot_state
#  ─────────────────────
#  this part is your follower_namespace
```

| Your config | `follower_namespace` value |
|-------------|---------------------------|
| Single pair, `namespace: franka_teleop` | `franka_teleop/follower` |
| Docker default | `franka_ros2_teleop/follower` |
| FR3 Duo left arm | `fr3_duo/left/follower` |

---

## 🔧 Troubleshooting

### Velocity columns are all zero

This was a bug in older versions. Make sure you are using the updated `data_collector_node.py` from this repo. The fix computes velocity from finite differences of EE position instead of reading `o_dp_ee_d` (which is always zero in gravity comp mode).

To verify the fix is working, check the save log — it prints velocity ranges on every save:

```
vel_vx  [-0.312, 0.421]  vy [-0.289, 0.398]  vz [-0.091, 0.104]
```

If all zeros, you are still running the old file.

### No data / topics not found

```bash
# Check the follower namespace is correct
ros2 topic list | grep franka_robot_state_broadcaster

# Re-launch with the correct namespace
ros2 launch franka_data_collector data_collector.launch.py \
    follower_namespace:=YOUR_NAMESPACE/follower
```

### Gripper columns are all zero

The gripper topic was not received. Check which namespace the gripper runs under:

```bash
ros2 topic list | grep gripper
```

Then pass the correct namespace to the gripper controller:

```bash
python3 scripts/gripper_controller.py --namespace franka_teleop/follower/franka_gripper
```

### `franka_msgs` not found inside container

```bash
# Inside the container
source /ros2_ws/install/setup.bash
# Then retry
```

### Data not appearing in `./franka_data/` on host

Make sure the directory exists **before** running `docker compose up`:

```bash
mkdir -p ./franka_data
docker compose -f docker-compose.with_collector.yml up --build
```

### Container name for `docker exec`

```bash
# List running containers to get the exact name
docker ps

# Then:
docker exec -it <CONTAINER_NAME> bash
```

---

## 📐 ROS 2 Topics Reference

### Subscribed by data collector

| Topic | Type | Data |
|-------|------|------|
| `/<ns>/franka_robot_state_broadcaster/measured_joint_states` | `sensor_msgs/JointState` | q, dq, tau |
| `/<ns>/franka_robot_state_broadcaster/external_joint_torques` | `sensor_msgs/JointState` | tau_ext |
| `/<ns>/franka_robot_state_broadcaster/robot_state` | `franka_msgs/FrankaRobotState` | O_T_EE (pose) |
| `/<ns>/franka_gripper/joint_states` | `sensor_msgs/JointState` | finger pos, vel, effort |
| `/franka_gripper/joint_states` | `sensor_msgs/JointState` | finger pos, vel, effort (fallback) |

### Used by gripper controller

| Topic / Action | Type | Purpose |
|----------------|------|---------|
| `/franka_gripper/move` | `franka_msgs/action/Move` | Open / move to width |
| `/franka_gripper/grasp` | `franka_msgs/action/Grasp` | Close with force |

---

<div align="center">

**Built for kinesthetic teaching of Franka FR3 robots.**  
Collected data is ready for use with imitation learning and trajectory learning pipelines.

</div>
