# njc_ros — Neural Jacobian Controller for ROS 2

ROS 2 Jazzy package that deploys the NJC inference pipeline as real-time controller nodes within the `data_collection` workspace.

## Prerequisites

- ROS 2 Jazzy
- Python 3.12+
- `data_collection` workspace built (`colcon build`)
- Kinova driver running (`launch_kinova`)

## Pip Dependencies

These are **not** managed by colcon — install them into the ROS 2 Python environment:

```bash
pip install -e ~/njc              # → from njc.inference import InferenceModel
pip install -e ~/dlopredict       # → from simplepredict.dataset_utils.spline_utils import fit_spline_unordered
```

## Build

```bash
cd ~/data_collection
colcon build --symlink-install --packages-select njc_ros
source install/setup.bash
```

## Launch

**Standalone** (assumes `launch_data_collection` is already running):

```bash
ros2 launch njc_ros njc_controller.launch.py model_dir:=/path/to/deploy/dir
```

**Full system** (launches data_collection_bringup + NJC nodes):

```bash
ros2 launch njc_ros njc_full_system.launch.py model_dir:=/path/to/deploy/dir
```

### Shell Aliases

Add to your `~/.bashrc`:

```bash
alias launch_njc='ros2 launch njc_ros njc_controller.launch.py'
alias launch_njc_full='ros2 launch njc_ros njc_full_system.launch.py'
```

## Parameters

All defaults are in `config/njc_params.yaml`.

### dlo_spline_node

| Parameter | Default | Description |
|-----------|---------|-------------|
| `num_kp` | 14 | Number of resampled keypoints |
| `momentum_weight` | 0.2 | Sorting momentum weight |
| `spline_k` | 3 | B-spline degree |
| `min_markers` | 10 | Minimum markers to process (skip frame if fewer) |
| `rate` | 30.0 | Publishing rate (Hz) |

### njc_controller_node

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model_dir` | **REQUIRED** | Path to model directory (config.yaml + checkpoint) |
| `checkpoint_name` | `checkpoint_best.pth` | Checkpoint filename |
| `controller_type` | `dls` | Controller: `dls`, `lyapunov`, `lyapunov_var` |
| `kp` | 1.0 | Proportional gain |
| `damping` | 0.1 | DLS damping lambda |
| `epsilon` | 0.01 | Manipulability threshold (Lyapunov variants) |
| `alpha` | 1.0 | Variance weight (lyapunov_var only) |
| `device` | `auto` | PyTorch device (`auto`, `cuda`, `cpu`) |
| `control_rate` | 30.0 | Control loop rate (Hz) |
| `num_kp` | 14 | Expected cable keypoint count |
| `max_linear_vel` | 0.5 | Linear velocity saturation (m/s) |
| `max_angular_vel` | 1.0 | Angular velocity saturation (rad/s) |
| `max_data_age_sec` | 0.2 | Watchdog: max input data age before stopping |

## Topic Interface

### Subscribed

| Topic | Type | Source |
|-------|------|--------|
| `/natnet/unlabeled_marker_data` | `visualization_msgs/Marker` | `natnet_pub_node` |
| `/end_effector_pose` | `geometry_msgs/PoseStamped` | `kinova_state_pub_node` |
| `/natnet/fixed_ee_pose` | `geometry_msgs/PoseStamped` | `natnet_pub_node` |
| `/njc/target_keypoints` | `my_msgs/PointsArray` | User / trajectory planner |

### Published

| Topic | Type | Source |
|-------|------|--------|
| `/njc/cable_keypoints` | `my_msgs/PointsArray` | `dlo_spline_node` |
| `/njc/cmd_vel` | `geometry_msgs/TwistStamped` | `njc_controller_node` |

## Data Flow

```
natnet_pub_node                     kinova_state_pub_node
  /natnet/unlabeled_marker_data       /end_effector_pose
  /natnet/fixed_ee_pose               (PoseStamped, base_link)
  (Marker POINTS, base_link)
        │                                    │
        ▼                                    │
  dlo_spline_node                            │
  (sort → spline fit → resample to N kp)     │
  /njc/cable_keypoints                       │
  (my_msgs/PointsArray, base_link)           │
        │                                    │
        ▼                                    ▼
  ┌──────────────────────────────────────────────┐
  │            njc_controller_node               │
  │  DLOProcessor → DLOBuffer → InferenceModel   │
  │  → Controller → TwistStamped                 │
  └──────────────────────────────────────────────┘
        │
        ▼
  /njc/cmd_vel (TwistStamped)
```

## Dependencies

- `natnet_pub_node` must be running (provides marker data + fixed gripper pose)
- `kinova_state_pub_node` must be running (provides EE pose via TF)
- Kinova driver must be running (provides TF tree)
- OptiTrack Motive must be streaming on the NatNet server

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `ModuleNotFoundError: njc` | Run `pip install -e ~/njc` in the ROS 2 Python env |
| `ModuleNotFoundError: simplepredict` | Run `pip install -e ~/dlopredict` |
| `FileNotFoundError: config.yaml not found` | Check `model_dir` points to a valid deploy directory |
| No `/njc/cable_keypoints` published | Check marker count — need at least `min_markers` (default 10) |
| Stale data warnings | Verify natnet_pub and kinova_state_pub are running and publishing |
| High latency / slow inference | Set `device:=cuda` explicitly; check GPU availability with `nvidia-smi` |
| Velocity output is zero | Ensure `/njc/target_keypoints` is being published |
