# Data Collection Repository - AI Agent Reference

> This document is intended for AI coding assistants (like Antigravity/Claude) to understand the architecture and workings of this ROS2 data collection system.

## Overview

This repository contains a **ROS2 Jazzy** workspace for collecting synchronized multi-modal sensor data from a robotic manipulation setup. The system captures:

- **Tactile sensing**: 4× DIGIT tactile sensors (D21118, D21122, D21123, D21124)
- **Motion capture**: OptiTrack system via NatNet protocol
- **Robot state**: Kinova Gen3 arm joint states and end-effector pose
- **Gripper**: Robotiq 2F-85 gripper

---

## Directory Structure

```
data_collection/
├── src/my_code/                    # Main ROS2 packages (focus here)
│   ├── data_collection_bringup/    # Launch files and configs
│   ├── digit_pub/                  # DIGIT tactile sensor publisher
│   ├── kinova_state_pub/           # Robot end-effector pose publisher
│   ├── natnet_pub/                 # OptiTrack motion capture publisher
│   ├── full_data_pub/              # [DEPRECATED] Synchronized data publisher
│   ├── robotiq_2f_85_driver/       # Robotiq gripper driver
│   ├── robotiq_msgs/               # Gripper message definitions
│   ├── robotiq_rviz_plugin/        # RViz plugin for gripper control
│   ├── my_msgs/                    # Custom message definitions
│   ├── natnet_client/              # NatNet SDK Python wrapper
│   └── ...
├── recordings/                     # Recorded rosbag files (MCAP format)
├── bag_data_collections/           # Organized bag data
├── archive/                        # Old/unused packages
└── README.md                       # User-facing data collection procedure
```

---

## Core ROS2 Nodes

### 1. `digit_pub_node` (digit_pub package)

**Purpose**: Publishes images from 4 DIGIT tactile sensors.

**Key files**:
- `digit_pub/digit_pub/digit_pub_node.py` - Main node
- `digit_pub/digit_pub/digit_array.py` - DIGIT hardware interface wrapper

**Topics published**:
| Topic | Type | Description |
|-------|------|-------------|
| `/digit/{SERIAL}/image_raw` | sensor_msgs/Image | Live tactile images (QVGA, 60fps) |
| `/digit/{SERIAL}/ref_image` | sensor_msgs/Image | Reference frame (TRANSIENT_LOCAL QoS) |

**Parameters** (in `data_collection_bringup.yaml`):
- `rate`: Publishing rate (default: 120 Hz)
- `resolution`: "QVGA" or "VGA"
- `fps`: "high" (60fps QVGA, 30fps VGA) or "low"
- `intensity`: LED intensity (0-15)
- `diff_with_ref`: Whether to publish difference images

**Commands**:
- Publish `"save_ref"` to `/digit_cmd` topic to capture new reference frames

---

### 2. `natnet_client_pub_node` (natnet_pub package)

**Purpose**: Connects to OptiTrack Motive software and publishes motion capture data.

**Key file**: `natnet_pub/natnet_pub/natnet_pub_node.py`

**Topics published**:
| Topic | Type | Description |
|-------|------|-------------|
| `/natnet/unlabeled_marker_data` | visualization_msgs/Marker | Unlabeled marker positions |
| `/natnet/base_link_pose` | geometry_msgs/PoseStamped | Robot base rigid body pose |
| `/natnet/robot_ee_pose` | geometry_msgs/PoseStamped | Robot end-effector rigid body |
| `/natnet/fixed_ee_pose` | geometry_msgs/PoseStamped | Fixed gripper rigid body (transformed to base_link) |

**TF broadcasts**:
- `base_link` → `natnet_world` (calibration transform)
- `natnet_world` → `natnet_robot_base`, `natnet_robot_ee`, `natnet_fixed_ee`

**Parameters**:
- `server_address`: OptiTrack PC IP
- `client_address`: This PC's IP
- `t_calib`, `q_calib`: Calibration transform (base_link ↔ natnet_world)
- `robot_base_rb_id`, `robot_ee_rb_id`, `fixed_ee_rb_id`: Rigid body IDs in Motive

---

### 3. `end_effector_pose_pub_node` (kinova_state_pub package)

**Purpose**: Publishes robot end-effector pose by looking up TF transform.

**Key file**: `kinova_state_pub/kinova_state_pub/kinova_state_pub_node.py`

**Topics published**:
| Topic | Type | Description |
|-------|------|-------------|
| `/end_effector_pose` | geometry_msgs/PoseStamped | EE pose in base_link frame |

**Parameters**:
- `base_frame`: "base_link"
- `end_effector_frame`: "end_effector_link"
- `publish_rate`: 500 Hz
- `timeout_sec`: TF lookup timeout

---

### 4. `full_data_pub` (full_data_pub package) — DEPRECATED

**Purpose**: Synchronized republishing of all sensor data with unified timestamps.

**Note**: User indicated this is no longer in use. The recording now uses throttled raw topics directly.

---

## Launch Files

### `data_collection_bringup.launch.py`
Launches all sensor nodes:
- `end_effector_pose_pub_node`
- `natnet_client_pub_node`
- `digit_pub_node`
- `robotiq_rviz.launch.py` (gripper driver + RViz plugin)

**Usage**: `ros2 launch data_collection_bringup data_collection_bringup.launch.py`

### `record.launch.py`
Records throttled topics to an MCAP bag file.

**Arguments**:
- `name`: Bag file name suffix
- `directory`: Save directory (default: `/home/rotem/data_collection/recordings/`)
- `duration`: Recording duration in seconds (default: 600)
- `rate_hz`: Throttle rate (default: 10 Hz)

**Topics recorded** (all throttled to `rate_hz`):
- `/digit/D2111*/image_raw_throttled`
- `/joint_states_throttled`
- `/natnet/unlabeled_marker_data_throttled`
- `/natnet/fixed_ee_pose_throttled`
- `/end_effector_pose_throttled`

---

## Configuration

**Main config file**: `data_collection_bringup/config/data_collection_bringup.yaml`

Contains parameters for:
- `digit_pub_node`: rate, resolution, fps, intensity
- `natnet_client_pub_node`: server/client IPs, calibration, rigid body IDs
- `full_data_pub`: (deprecated)

---

## Hardware Setup

### DIGIT Sensors
- 4 DIGIT tactile sensors connected via USB
- Serial numbers: D21118, D21122, D21123, D21124
- Reset script: `./reset_digits.sh` (if USB enumeration issues)

### OptiTrack
- Server IP: Check `data_collection_bringup.yaml` → `server_address`
- Requires Motive software running on separate Windows PC

### Kinova Gen3
- Launched separately via `launch_kinova` alias
- Provides `/joint_states` and TF tree

### Robotiq 2F-85
- Controlled via RViz plugin or `/robotiq_2f_urcap_adapter/command` topic

---

## Common Shell Aliases (expected in user's bashrc)

```bash
launch_kinova      # Launch Kinova driver + MoveIt
launch_data_collection  # ros2 launch data_collection_bringup data_collection_bringup.launch.py
run_bag            # ros2 launch data_collection_bringup record.launch.py
r2 / setupr2       # Source ROS2 workspace
```

---

## Data Flow Diagram

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│  DIGIT x4   │     │  OptiTrack  │     │ Kinova Gen3 │
└──────┬──────┘     └──────┬──────┘     └──────┬──────┘
       │                   │                   │
       ▼                   ▼                   ▼
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│  digit_pub  │     │  natnet_pub │     │kinova_state │
│             │     │             │     │    _pub     │
└──────┬──────┘     └──────┬──────┘     └──────┬──────┘
       │                   │                   │
       ▼                   ▼                   ▼
┌────────────────────────────────────────────────────┐
│              topic_tools/throttle                   │
│         (reduces all topics to 10 Hz)               │
└────────────────────────┬───────────────────────────┘
                         │
                         ▼
┌────────────────────────────────────────────────────┐
│              rosbag2_recorder (MCAP)               │
└────────────────────────────────────────────────────┘
```

---

## Key Implementation Details

1. **QoS Profiles**:
   - Live data: `BEST_EFFORT` / `VOLATILE`
   - Reference images: `RELIABLE` / `TRANSIENT_LOCAL` (latched)
   - Published synced data: `RELIABLE` / `VOLATILE`

2. **Calibration**:
   - `t_calib` and `q_calib` define the transform from `base_link` to `natnet_world`
   - Computed via external calibration procedure

3. **Threading**:
   - `natnet_pub` uses `MultiThreadedExecutor` implicitly through NatNet SDK callbacks
   - Other nodes use single-threaded executor

4. **Recording format**: MCAP (efficient, supports all ROS2 message types)

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| DIGIT not detected | Run `./reset_digits.sh`, replug USB |
| NatNet not connecting | Check IP addresses, ensure Motive is streaming |
| TF lookup fails | Ensure Kinova driver is running first |
| Missing markers | Check `num_markers` parameter matches Motive setup |
