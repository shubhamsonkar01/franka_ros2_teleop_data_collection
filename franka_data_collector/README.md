# franka_data_collector

A ROS 2 package that collects 42-dimensional state data from a Franka follower robot during teleoperation and saves it as **CSV** and/or **NumPy (.npy)** files.

## Data Format

Each recorded timestep contains **42 values**:

| Index | Field | Description | Unit |
|-------|-------|-------------|------|
| 0 | `timestamp_ns` | ROS clock timestamp | ns |
| 1–7 | `q1`…`q7` | Joint positions | rad |
| 8–14 | `dq1`…`dq7` | Joint velocities | rad/s |
| 15–21 | `tau1`…`tau7` | Measured joint torques | Nm |
| 22–28 | `tau_ext1`…`tau_ext7` | External joint torques | Nm |
| 29–31 | `pos_x`, `pos_y`, `pos_z` | EE Cartesian position | m |
| 32–35 | `quat_x`, `quat_y`, `quat_z`, `quat_w` | EE orientation (quaternion) | – |
| 36–41 | `vel_vx`…`vel_wz` | EE Cartesian velocity (linear + angular) | m/s, rad/s |

## Setup

### Option A — Run inside the Docker container (Recommended)

This is the easiest path. You place `franka_data_collector` next to `franka_ros2_teleop` inside the existing Docker image.

**Step 1 — Copy the package into the repo:**
```
franka_ros2_teleop/           ← your existing repo root
├── src/
│   └── ...
├── franka_data_collector/    ← ADD THIS FOLDER HERE
├── Dockerfile.updated        ← ADD THIS FILE HERE
├── docker-compose.yml        ← your existing file (keep as-is for teleop only)
└── docker-compose.with_collector.yml  ← ADD THIS FILE HERE
```

**Step 2 — Create the output directory on your host:**
```bash
mkdir -p ./franka_data
```

**Step 3 — Start everything with a single command:**
```bash
docker compose -f docker-compose.with_collector.yml up --build
```

This starts **both** the teleoperation node and the data collector simultaneously.
Data appears in `./franka_data/` on your host machine.

---

### Option B — Build in your own ROS 2 workspace

```bash
# Copy franka_data_collector into your workspace src directory
cp -r franka_data_collector/ ~/ros2_ws/src/

# Install Python dependency (numpy)
pip3 install numpy --break-system-packages

# Build
cd ~/ros2_ws
colcon build --packages-select franka_data_collector --symlink-install
source install/setup.bash
```

**Then, in a second terminal while teleoperation is running:**
```bash
# Default config (namespace = franka_teleop/follower)
ros2 launch franka_data_collector data_collector.launch.py

# Custom namespace (check your fr3_teleop_config.yaml pairs.namespace)
ros2 launch franka_data_collector data_collector.launch.py \
    follower_namespace:=YOUR_PAIR_NAMESPACE/follower \
    output_dir:=/home/youruser/robot_data
```

---

## Finding Your `follower_namespace`

Look at your `fr3_teleop_config.yaml` or `docker-compose.yml`:

```yaml
# fr3_teleop_config.yaml
pairs:
  - namespace: franka_teleop   # ← this
    leader:
      robot_ip: ...
    follower:
      robot_ip: ...
```

The follower namespace = `<pair_namespace>/follower`

| Config | follower_namespace |
|--------|--------------------|
| `pairs[0].namespace: franka_teleop` | `franka_teleop/follower` |
| `pairs[0].namespace: franka_ros2_teleop` (docker default) | `franka_ros2_teleop/follower` |
| FR3 Duo left arm | `fr3_duo/left/follower` |

**To verify at runtime:**
```bash
# List all active topics to find the right namespace
ros2 topic list | grep franka_robot_state
```

---

## ROS Topics Subscribed

Under `/<follower_namespace>/`:

| Topic | Message Type | Data extracted |
|-------|-------------|----------------|
| `franka_robot_state_broadcaster/measured_joint_states` | `sensor_msgs/JointState` | joint pos, vel, measured torque |
| `franka_robot_state_broadcaster/external_joint_torques` | `sensor_msgs/JointState` | external torques |
| `franka_robot_state_broadcaster/robot_state` | `franka_msgs/FrankaRobotState` | EE pose (4x4), Cartesian vel, ext torques |

---

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `follower_namespace` | `franka_teleop/follower` | ROS namespace of the follower robot |
| `output_dir` | `/tmp/franka_data` | Directory to save files |
| `save_csv` | `true` | Save CSV file |
| `save_npy` | `true` | Save NPY file |
| `buffer_size` | `100000` | Flush to disk every N samples (~100s at 1kHz) |
| `auto_save_interval_sec` | `30.0` | Periodic save interval (0 = disable) |
| `use_franka_robot_state` | `true` | Subscribe to FrankaRobotState (needs franka_msgs) |

---

## Output Files

Files are named: `franka_follower_<session>_<timestamp>_<suffix>.{npy,csv}`

- `_autosave` — periodic saves during recording
- `_partial` — buffer-full flushes
- `_final` — save on Ctrl+C or node shutdown

**Stop recording:**  Press `Ctrl+C` in the terminal running the collector. The final save happens automatically.

---

## Loading Data in Python

```python
import numpy as np

# Load NPY (faster)
data = np.load('/tmp/franka_data/franka_follower_final.npy')

# Or load CSV
data = np.loadtxt('/tmp/franka_data/franka_follower_final.csv',
                  delimiter=',', skiprows=1)

# Shape: (N_samples, 42)
print(data.shape)

# Access fields
timestamps  = data[:, 0]          # ns
joint_pos   = data[:, 1:8]        # rad   (7,)
joint_vel   = data[:, 8:15]       # rad/s (7,)
measured_τ  = data[:, 15:22]      # Nm    (7,)
ext_τ       = data[:, 22:29]      # Nm    (7,)
cart_pos    = data[:, 29:32]      # m     (3,)
quaternion  = data[:, 32:36]      # xyzw  (4,)
cart_vel    = data[:, 36:42]      # m/s, rad/s (6,)

# Convert timestamps to seconds from start
t_sec = (timestamps - timestamps[0]) * 1e-9
```

---

## Inspecting / Visualizing Data

Use the included `franka_data_inspector.py` script (no ROS needed):

```bash
# Summary of a single file
python3 franka_data_inspector.py --file /tmp/franka_data/franka_follower_20250101_*.npy

# Merge all files in a directory + summary
python3 franka_data_inspector.py --dir /tmp/franka_data --merge

# Show plots
python3 franka_data_inspector.py --file data.npy --plot

# Export merged NPY
python3 franka_data_inspector.py --dir /tmp/franka_data --merge --export merged.npy

# Convert NPY to CSV
python3 franka_data_inspector.py --file data.npy --export data.csv
```

---

## Troubleshooting

### "Zero cartesian position / external torques in output"
The `franka_robot_state_broadcaster/robot_state` topic is not being received.

1. Check if `fake_hardware: true` is set — fake hardware won't publish robot state.
2. Verify the topic exists: `ros2 topic list | grep robot_state`
3. Try setting `use_franka_robot_state:=false` to still collect joint data.

### "No data published / node subscribes but nothing collected"
Check the namespace is correct:
```bash
ros2 topic list | grep franka_robot_state_broadcaster
# Should show: /YOUR_NAMESPACE/follower/franka_robot_state_broadcaster/...
```

Then re-launch with the correct namespace:
```bash
ros2 launch franka_data_collector data_collector.launch.py \
    follower_namespace:=YOUR_CORRECT_NAMESPACE/follower
```

### "franka_msgs not found"
The collector still works but external torques and Cartesian data will be zeros.
Set `use_franka_robot_state:=false` and make sure you have `franka_ros2` built.

### Inside Docker: data not appearing on host
Ensure the bind mount in `docker-compose.with_collector.yml` matches your host path and that `./franka_data` directory exists before running `docker compose up`.
