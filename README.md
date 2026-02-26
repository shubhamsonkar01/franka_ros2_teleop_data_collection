<div align="center">

<h1>🤖 franka_ros2_teleop</h1>

<p><strong>High-frequency leader–follower teleoperation framework for Franka FR3 robots with learning-ready state logging at ~1 kHz.</strong></p>

[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Docker](https://img.shields.io/badge/Docker-Ready-2496ED?logo=docker&logoColor=white)](https://www.docker.com/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green?logo=apache)](LICENSE)
[![Franka FR3](https://img.shields.io/badge/Robot-Franka%20FR3-orange)](https://www.franka.de/)
[![Python 3.10](https://img.shields.io/badge/Python-3.10+-yellow?logo=python&logoColor=white)](https://www.python.org/)

<br/>

*Kinesthetic teaching · End-effector velocity · Gripper recording · Docker-first · ML-ready*

</div>

---

## 📋 Table of Contents

- [Research Motivation](#-research-motivation)
- [What This Repository Does](#-what-this-repository-does)
- [Design Principles](#-design-principles)
- [Repository Structure](#-repository-structure)
- [Where to Place the New Files](#-where-to-place-the-new-files)
- [Prerequisites — What You Need Before Starting](#-prerequisites--what-you-need-before-starting)
- [Hardware Setup](#-hardware-setup)
- [Quick Start — Step by Step](#-quick-start--step-by-step)
- [Data Collection](#-data-collection)
- [Gripper Control](#-gripper-control)
- [Loading Data in Python](#-loading-data-in-python)
- [Configuration Reference](#-configuration-reference)
- [Reproducibility](#-reproducibility)
- [Troubleshooting](#-troubleshooting)
- [Contributing](#-contributing)
- [Citation](#-citation)

---

## 🔬 Research Motivation

High-frequency kinesthetic demonstrations are critical for:

- Learning stable manipulation policies from human demonstration
- Modeling human teaching intent and motion primitives
- Neural ODE trajectory modeling of continuous robot dynamics
- Safe control synthesis using CLF-CBF constraints
- Imitation learning and behaviour cloning pipelines

This framework captures physically consistent robot state at ~1 kHz during gravity-compensation teaching — ensuring downstream learning algorithms receive **high-fidelity, temporally dense state data** including correct end-effector velocity (computed via finite differences, not the controller's desired velocity which is always zero during kinesthetic teaching).

---

## 🎯 What This Repository Does

```
You physically move the leader robot
        ↓
The follower robot mirrors your motion in real time
        ↓
Every joint angle, torque, EE pose, EE velocity, and gripper state
is recorded at ~1 kHz to CSV and NumPy files
        ↓
Files are ready for imitation learning / Neural ODE / CLF-CBF pipelines
```

In one diagram:

```
  ┌─────────────┐   teleoperation   ┌──────────────┐
  │  Leader FR3 │ ────────────────► │ Follower FR3 │
  │  (you move) │                   │  (mirrors)   │
  └─────────────┘                   └──────┬───────┘
                                           │ ~1 kHz state
                                           ▼
                                  franka_data/*.npy
                                  franka_data/*.csv
                                  (46 values / timestep)
```

---

## 🧩 Design Principles

| Principle | Implementation |
|-----------|----------------|
| **Deterministic recording** | All state sampled synchronously at 1 kHz via a single timer callback |
| **Controller-independent velocity** | EE velocity computed from finite differences of measured position — not from the desired velocity field which is always zero in gravity comp mode |
| **Modular ROS 2 package** | `franka_data_collector` is a self-contained package decoupled from the teleop controller |
| **Separation of concerns** | Teleoperation and data logging are independent processes |
| **Docker-first reproducibility** | Single `docker compose` command brings up the full system — no manual ROS installation |
| **Learning-ready output** | 46-dimensional state vector matches standard imitation learning input formats |

---

## 📁 Repository Structure

```
franka_ros2_teleop/
│
├── 📂 franka_data_collector/          ROS 2 data collection package
│   ├── franka_data_collector/
│   │   ├── data_collector_node.py     ⭐ Main collector (velocity fix + gripper)
│   │   └── __init__.py
│   ├── launch/
│   │   └── data_collector.launch.py   Launch file
│   ├── config/
│   │   └── data_collector_params.yaml Parameters
│   ├── CMakeLists.txt
│   └── package.xml
│
├── 📂 scripts/                        Utility scripts
│   └── gripper_controller.py          ⭐ Interactive gripper open/close
│
├── 📂 franka_data/                    Recorded data saved here (auto-created)
│   └── *.npy / *.csv
│
├── 📂 src/                            C++ teleoperation controllers
│   ├── teleop_leader_controller.cpp
│   ├── teleop_follower_controller.cpp
│   ├── teleop_coordinator.cpp
│   └── teleop_gripper_node.cpp
│
├── 📂 include/franka_ros2_teleop/     C++ headers
│
├── 📂 launch/                         Teleop launch files
│   ├── teleop.launch.py
│   └── teleop_single_robot.launch.py
│
├── 📂 config/                         Robot configuration
│   ├── fr3_teleop_config.yaml         Single pair setup
│   ├── fr3_duo_teleop_config.yaml     Dual arm setup
│   └── teleop_controllers.yaml
│
├── franka_data_inspector.py           Inspect and plot recorded data
├── Dockerfile                         Base teleop image
├── Dockerfile.updated                 Extended image (teleop + collector)
├── docker-compose.yml                 Teleop only
└── docker-compose.with_collector.yml  Teleop + data collection  ← use this
```

---

## 📂 Where to Place the New Files

You downloaded two files. Here is exactly where they go:

```
franka_ros2_teleop/
│
├── franka_data_collector/
│   └── franka_data_collector/
│       └── data_collector_node.py    ← 🔴 REPLACE the existing file
│
└── scripts/
    └── gripper_controller.py         ← 🟢 CREATE this folder, place file here
```

```bash
# Replace the collector node
cp data_collector_node.py \
   franka_ros2_teleop/franka_data_collector/franka_data_collector/data_collector_node.py

# Create the scripts folder and place the gripper controller
mkdir -p franka_ros2_teleop/scripts
cp gripper_controller.py franka_ros2_teleop/scripts/gripper_controller.py
```

---

## ✅ Prerequisites — What You Need Before Starting

> **If you are new to this stack**, go through this checklist before touching any code. Each item links to installation instructions.

### Hardware

- [ ] 2× Franka FR3 robots (or 1 for single-arm mode)
- [ ] 1× PC/workstation connected to both robots over Ethernet
- [ ] Both robots powered on — indicator light should be **white** or **blue**

### Software on your PC

| Software | Why you need it | How to get it |
|----------|----------------|---------------|
| **Docker** | Runs the entire ROS 2 stack — no ROS installation needed | [docs.docker.com/get-docker](https://docs.docker.com/get-docker/) |
| **Docker Compose** | Starts multiple containers together | Comes with Docker Desktop; or `sudo apt install docker-compose-plugin` |
| **Git** | Clone this repository | `sudo apt install git` |
| **Python 3.10+** | Inspect data files offline (no ROS needed) | `sudo apt install python3` |
| **numpy** | Load `.npy` data files | `pip install numpy` |

> **Windows users:** Install [Docker Desktop for Windows](https://docs.docker.com/desktop/install/windows-install/) and run all commands in PowerShell or WSL2.

### Network check

Your PC must be able to reach both robots:

```bash
ping 192.168.1.12   # leader — should reply with times like "64 bytes from ..."
ping 192.168.1.15   # follower — same
```

If either fails, check the Ethernet cable and ensure your PC's IP is in the same subnet (e.g. `192.168.1.x`).

### Franka Desk check

Open each robot's web interface in your browser and confirm:

- Leader:   `http://192.168.1.12` → no errors → joints unlocked → FCI enabled
- Follower: `http://192.168.1.15` → no errors → joints unlocked → FCI enabled

The robot light should be **blue** (FCI active). Joints should move freely when you gently push them — this is gravity compensation mode.

---

## 🔌 Hardware Setup

```
┌──────────────────────────────────────────────────────────────────┐
│                        Network Switch / Router                    │
│                                                                  │
│   ┌──────────────┐              ┌──────────────┐                 │
│   │  Leader FR3  │              │ Follower FR3 │                 │
│   │ 192.168.1.12 │              │ 192.168.1.15 │                 │
│   │  (you move)  │              │  (records)   │                 │
│   └──────┬───────┘              └──────┬───────┘                 │
│          │  Ethernet                   │  Ethernet               │
│          └──────────────┬──────────────┘                         │
│                         │                                        │
│                ┌────────┴─────────┐                              │
│                │    Your PC        │                              │
│                │   Docker Host     │                              │
│                │                   │                              │
│                │  ./franka_data/   │ ← data files appear here    │
│                └───────────────────┘                              │
└──────────────────────────────────────────────────────────────────┘
```

---

## 🚀 Quick Start — Step by Step

> Follow every step in order. Each step builds on the previous one.

---

### Step 1 — Clone the repository

```bash
git clone git@github.com:shubhamsonkar01/franka_ros2_teleop_data_collection.git
cd franka_ros2_teleop_data_collection
```

---

### Step 2 — Set your robot IPs

Open `docker-compose.with_collector.yml` in any text editor and update the two IP addresses:

```yaml
x-teleop-config:
  pairs:
    - namespace: franka_teleop
      leader:
        robot_ip: 192.168.1.12    # ← your leader robot IP
      follower:
        robot_ip: 192.168.1.15    # ← your follower robot IP
```

Save the file.

---

### Step 3 — Create the data output folder

```bash
mkdir -p ./franka_data
```

Data files will appear here automatically during and after recording.

---

### Step 4 — Build the Docker image

```bash
docker compose -f docker-compose.with_collector.yml build
```

> ⏳ This downloads ROS 2, franka_ros2, libfranka, and builds the workspace. **First build takes 5–15 minutes.** Subsequent builds are fast unless you change the Dockerfile.

---

### Step 5 — Prepare the robots

Do this for **both** robots before launching:

1. Open `http://<robot_ip>` in your browser (e.g. `http://192.168.1.15`)
2. Clear any errors shown in Franka Desk
3. Click the lock icon to **unlock the joints**
4. Enable **FCI** (Franka Control Interface) — the robot light turns blue
5. Verify the joints move freely when you gently push them (gravity comp mode is active)

---

### Step 6 — Launch everything

```bash
docker compose -f docker-compose.with_collector.yml up
```

Wait until you see this in the terminal output:

```
[franka_data_collector] FrankaDataCollector started (STATE_DIM=46)
[franka_data_collector]   Velocity : computed from finite diff of O_T_EE
[franka_data_collector]   Gripper  : cols 42-45
```

**Recording is now active.** Pick up and move the leader robot — the follower mirrors your motion and all 46 state values are saved every millisecond.

---

### Step 7 — Stop and save

Press `Ctrl+C` in the terminal. The collector saves automatically:

```
[franka_data_collector] Shutdown — saving final data...
[franka_data_collector] Saved 9546 rows → franka_data/franka_follower_..._final.npy
[franka_data_collector]   vel_vx [-0.312, 0.421]  vy [-0.289, 0.398]  vz [-0.091, 0.104]
```

Your files are now in `./franka_data/` on your PC. Done.

---

## 📊 Data Collection

### What gets recorded

Every ~1 ms the collector writes **46 values** per row:

| Columns | Field | Description | Unit |
|---------|-------|-------------|------|
| `[0]` | `timestamp_ns` | ROS clock timestamp | ns |
| `[1–7]` | `q1`…`q7` | Joint positions | rad |
| `[8–14]` | `dq1`…`dq7` | Joint velocities | rad/s |
| `[15–21]` | `tau1`…`tau7` | Measured joint torques | Nm |
| `[22–28]` | `tau_ext1`…`tau_ext7` | External joint torques | Nm |
| `[29–31]` | `pos_x`, `pos_y`, `pos_z` | EE Cartesian position | m |
| `[32–35]` | `quat_x`, `quat_y`, `quat_z`, `quat_w` | EE orientation | — |
| `[36–41]` | `vel_vx`, `vel_vy`, `vel_vz`, `vel_wx`, `vel_wy`, `vel_wz` | EE Cartesian velocity ✅ | m/s, rad/s |
| `[42]` | `gripper_left_pos` | Left finger position | m |
| `[43]` | `gripper_right_pos` | Right finger position | m |
| `[44]` | `gripper_vel` | Finger velocity (mean) | m/s |
| `[45]` | `gripper_effort` | Finger force (mean) | N |

> **About velocity (cols 36–41):** During kinesthetic teaching the robot controller is in gravity compensation — it does not issue commanded velocities, so the `o_dp_ee_d` field in `FrankaRobotState` is always zero. This collector instead computes EE velocity from finite differences of the measured `O_T_EE` position at 1 kHz, producing accurate smooth velocities for learning.

### Output file names

```
franka_data/
├── franka_follower_20260223_163230_final_20260223_163240.npy    ← use this one
├── franka_follower_20260223_163230_final_20260223_163240.csv    ← same, CSV format
├── franka_follower_20260223_163230_autosave_20260223_163210.npy ← periodic checkpoint
└── franka_follower_20260223_163230_partial_20260223_162950.npy  ← buffer-full flush
```

Use the `_final` files. The others are safety copies.

### Inspecting data (no ROS needed)

```bash
python3 franka_data_inspector.py --file franka_data/franka_follower_*_final.npy

# Merge all files from a session and show plots
python3 franka_data_inspector.py --dir franka_data/ --merge --plot

# Export everything into one file
python3 franka_data_inspector.py --dir franka_data/ --merge --export session1.npy
```

---

## 🤏 Gripper Control

Run the gripper controller in a **second terminal** while the main teleop is running. This lets you open and close the gripper during a recording — essential for pick-and-place demonstrations.

### Open a shell in the running container

```bash
# Find the container name
docker ps

# Open a shell inside it
docker exec -it <CONTAINER_NAME> bash

# Run the gripper controller
python3 /ros2_ws/src/franka_ros2_teleop/scripts/gripper_controller.py
```

### Keyboard controls

```
==========================================
  Franka Gripper Controller
==========================================
  o          →  OPEN  (full open, 8 cm)
  c          →  CLOSE (grasp, 20 N force)
  w <value>  →  open to specific width (m)
                e.g.  w 0.05  →  5 cm gap
  f <value>  →  set grasp force in N
                e.g.  f 40    →  40 N grip
  q          →  quit
==========================================
```

### One-shot commands

```bash
python3 scripts/gripper_controller.py --open
python3 scripts/gripper_controller.py --close
python3 scripts/gripper_controller.py --width 0.04   # half open
python3 scripts/gripper_controller.py --close --force 40
```

### Reading gripper state from recorded data

```python
total_width = data[:, 42] + data[:, 43]   # left + right finger (meters)

is_open     = total_width > 0.07   # hand fully open
is_grasping = total_width < 0.02   # holding an object
```

---

## 🐍 Loading Data in Python

```python
import numpy as np

# Load (no ROS required)
data = np.load('franka_data/franka_follower_final.npy')
print(data.shape)   # (N_samples, 46)

# Unpack all columns
timestamp   = data[:, 0]       # nanoseconds
joint_pos   = data[:, 1:8]     # rad    shape (N, 7)
joint_vel   = data[:, 8:15]    # rad/s  shape (N, 7)
torque      = data[:, 15:22]   # Nm     shape (N, 7)
torque_ext  = data[:, 22:29]   # Nm     shape (N, 7)
ee_pos      = data[:, 29:32]   # m      shape (N, 3)  [x, y, z]
ee_quat     = data[:, 32:36]   #        shape (N, 4)  [qx, qy, qz, qw]
ee_vel      = data[:, 36:42]   # m/s    shape (N, 6)  [vx, vy, vz, wx, wy, wz]
gripper_l   = data[:, 42]      # m      left finger position
gripper_r   = data[:, 43]      # m      right finger position
gripper_vel = data[:, 44]      # m/s
gripper_f   = data[:, 45]      # N

# Time axis
t = (timestamp - timestamp[0]) * 1e-9   # seconds from start

# Quick summary
print(f"Duration  : {t[-1]:.2f} s")
print(f"Samples   : {len(data)}")
print(f"Frequency : {len(data)/t[-1]:.0f} Hz")
print(f"EE vel vx : [{ee_vel[:,0].min():.3f}, {ee_vel[:,0].max():.3f}] m/s")

# Detect grasp events
total_width = gripper_l + gripper_r
grasping    = total_width < 0.02
print(f"Grasping  : {grasping.sum()} samples ({grasping.sum()/len(data)*100:.1f}%)")
```

---

## ⚙️ Configuration Reference

### Robot IPs

Edit `docker-compose.with_collector.yml`:

```yaml
x-teleop-config:
  pairs:
    - namespace: franka_teleop
      leader:
        robot_ip: 192.168.1.12
      follower:
        robot_ip: 192.168.1.15
```

### Collector parameters

Edit `franka_data_collector/config/data_collector_params.yaml`:

```yaml
franka_data_collector:
  ros__parameters:
    follower_namespace: "franka_teleop/follower"   # must match your pair namespace + /follower
    output_dir: "/tmp/franka_data"                 # container path, mounted to ./franka_data on host
    save_csv: true
    save_npy: true
    buffer_size: 100000                            # ~100 s at 1 kHz before auto-flush
    auto_save_interval_sec: 30.0                   # checkpoint every 30 s
    use_franka_robot_state: true
    vel_ema_alpha: 0.5                             # velocity smoothing  (0 = raw,  1 = frozen)
```

### Finding your `follower_namespace`

```bash
# While teleop is running:
ros2 topic list | grep robot_state

# You will see something like:
# /franka_teleop/follower/franka_robot_state_broadcaster/robot_state
#  ─────────────────────
#  this part is your follower_namespace
```

| Config setup | `follower_namespace` value |
|--------------|---------------------------|
| `namespace: franka_teleop` | `franka_teleop/follower` |
| Docker default | `franka_ros2_teleop/follower` |
| FR3 Duo left arm | `fr3_duo/left/follower` |

---

## 🔁 Reproducibility

Everything runs inside Docker. There is no manual ROS 2 installation, no workspace sourcing, no dependency hunting.

**Complete setup from zero to recording in 5 commands:**

```bash
# 1. Clone
git clone git@github.com:shubhamsonkar01/franka_ros2_teleop_data_collection.git
cd franka_ros2_teleop_data_collection

# 2. Create data folder
mkdir -p ./franka_data

# 3. Set your robot IPs in docker-compose.with_collector.yml  (open in any text editor)

# 4. Build
docker compose -f docker-compose.with_collector.yml build

# 5. Run
docker compose -f docker-compose.with_collector.yml up
```

**Pinned software versions:**

| Component | Version |
|-----------|---------|
| ROS 2 | Humble |
| franka_ros2 | v2.0.4 |
| franka_description | 1.0.2 |
| libfranka | 0.16.0 |
| Python | 3.10 |
| numpy | latest stable |

The system is fully deterministic given identical hardware, network, and these software versions.

---

## 🔧 Troubleshooting

### Velocity is all zero in saved files

You are running the original `data_collector_node.py`. Replace it with the one from this repo. When the fix is active, the terminal prints velocity ranges on every save:

```
vel_vx [-0.312, 0.421]  vy [-0.289, 0.398]  vz [-0.091, 0.104]
```

All zeros means the old file is still in place.

---

### No data collected / topics not found

```bash
# Check the correct namespace
ros2 topic list | grep franka_robot_state_broadcaster

# Relaunch with the correct one
ros2 launch franka_data_collector data_collector.launch.py \
    follower_namespace:=YOUR_NAMESPACE/follower
```

---

### Gripper columns are all zero

Check which namespace the gripper node uses:

```bash
ros2 topic list | grep gripper
```

Pass it explicitly:

```bash
python3 scripts/gripper_controller.py --namespace franka_teleop/follower/franka_gripper
```

---

### Data files not appearing in `./franka_data/` on host

The bind mount requires the directory to exist before launch:

```bash
mkdir -p ./franka_data
docker compose -f docker-compose.with_collector.yml up
```

---

### Docker build fails

```bash
docker compose -f docker-compose.with_collector.yml down
docker system prune -f
docker compose -f docker-compose.with_collector.yml build --no-cache
```

---

### Robot light is red / joints are locked

Go to `http://<robot_ip>` in your browser, clear all errors in Franka Desk, unlock the joints, and re-enable FCI before launching.

---

### `franka_msgs` not found inside container

```bash
# Inside the container, source the workspace
source /ros2_ws/install/setup.bash
```

---

## 📐 ROS 2 Topics Reference

### Subscribed by the data collector

| Topic | Message type | What is extracted |
|-------|-------------|-------------------|
| `/<ns>/franka_robot_state_broadcaster/measured_joint_states` | `sensor_msgs/JointState` | q, dq, tau |
| `/<ns>/franka_robot_state_broadcaster/external_joint_torques` | `sensor_msgs/JointState` | tau_ext |
| `/<ns>/franka_robot_state_broadcaster/robot_state` | `franka_msgs/FrankaRobotState` | O_T_EE (4×4 pose) |
| `/<ns>/franka_gripper/joint_states` | `sensor_msgs/JointState` | finger pos, vel, effort |
| `/franka_gripper/joint_states` | `sensor_msgs/JointState` | same — fallback topic |

### Used by the gripper controller

| Action server | Message type | Purpose |
|---------------|-------------|---------|
| `/franka_gripper/move` | `franka_msgs/action/Move` | Open / move to width |
| `/franka_gripper/grasp` | `franka_msgs/action/Grasp` | Close with force control |

---

## 🤝 Contributing

Contributions are welcome.

If you find a bug or want to extend functionality:

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/your-feature`
3. Commit with clear messages: `git commit -m "Add: description of change"`
4. Push and open a pull request

Please ensure Python files pass `flake8` and `pep257` linting before submitting.

---

## 📄 Citation

If you use this framework in academic work, please cite:

```bibtex
@software{franka_ros2_teleop_data_collection,
  author    = {Shubham Sonkar},
  title     = {franka\_ros2\_teleop: High-frequency Kinesthetic Teaching and Data Collection for Franka FR3},
  year      = {2026},
  url       = {https://github.com/shubhamsonkar01/franka_ros2_teleop_data_collection},
}
```

---

<div align="center">

Built for kinesthetic teaching and robot learning research on Franka FR3.<br/>
Collected data is ready for imitation learning, Neural ODE, and CLF-CBF safety pipelines.

</div>
