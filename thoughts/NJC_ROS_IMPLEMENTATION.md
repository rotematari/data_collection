# NJC ROS Implementation — What Was Built and How It Works

Branch: `ROT-22/deploy-njc-ros` | Issues: ROT-22 through ROT-26

---

## What is this?

Two new ROS 2 nodes that bring the NJC (Neural Jacobian Controller) from a standalone Python script into the live robot system. Together they close the loop: **see the cable** (OptiTrack markers) → **predict the Jacobian** (neural network) → **compute a velocity command** (controller) → **move the robot** (TwistStamped).

---

## The Two Nodes

### 1. `dlo_spline_node` — "Make sense of the marker cloud"

**File**: `njc_ros/dlo_spline_node.py`

**Problem it solves**: OptiTrack gives us an unordered cloud of 3D marker positions. The NJC model expects exactly 14 ordered, evenly-spaced keypoints along the cable.

**What it does each tick (30 Hz)**:
```
Raw marker cloud (unordered, variable count)
    │
    ├─ Skip if fewer than 10 markers
    ├─ Skip if data is older than 100ms
    │
    ▼
fit_spline_unordered()          ← from simplepredict
    │  1. Sort markers into wire order (nearest-neighbor + momentum)
    │  2. Fit B-spline through sorted points
    │  3. Resample spline to exactly 14 equidistant points
    │
    ▼
Publish /njc/cable_keypoints (PointsArray, 14 points, base_link frame)
```

**Design choice**: The Linear issue described a manual 3-step pipeline (`sort_points_from_origin` → `splprep` → `splev`). Instead, we use `fit_spline_unordered()` which wraps all three steps in a single call. Same result, less code to maintain.

**Key parameter**: `min_markers` (default 10) — if OptiTrack sees fewer markers than this (occlusion, reflections), the frame is silently dropped. This prevents garbage-in-garbage-out.

---

### 2. `njc_controller_node` — "Predict and act"

**File**: `njc_ros/njc_controller_node.py`

**Problem it solves**: Given the current cable shape and a target cable shape, compute what velocity to send to the robot end-effector.

**What it does each tick (30 Hz)**:
```
/njc/cable_keypoints      ──┐
/end_effector_pose         ──┤  (all must be present and fresh)
/natnet/fixed_ee_pose      ──┤
/njc/target_keypoints      ──┘
    │
    ├─ Watchdog: skip if ANY input is older than 200ms
    │
    ▼
Build raw observation dict
    │  marker_positions (14, 3)
    │  ee_position (3,) + ee_orientation (4, xyzw)
    │  fixed_position (3,) + fixed_orientation (4, xyzw)
    │
    ▼
DLOProcessor.prepare_dlo_observation()
    │  Converts to fixed-gripper frame
    │  Computes edges, curvatures, 6D rotation
    │  Returns DLOStateMsg
    │
    ▼
DLOBuffer.append()
    │  Accumulates temporal context (1 or 2 frames depending on model config)
    │  Waits until buffer is full before first inference
    │
    ▼
DLOBuffer.to_tensor() → add batch dim → InferenceModel.predict()
    │  Returns JacobianMsg: J shape (42, 6)
    │  Maps cable keypoint velocities → EEF velocity
    │
    ▼
Controller.compute_command(J, s_current, s_target)
    │  DLS: q̇ = (JᵀJ + λ²I)⁻¹ Jᵀ (kp · error)
    │  Returns CMDMsg: linear (3,) + angular (3,)
    │
    ▼
Velocity saturation → clamp to max_linear_vel / max_angular_vel
    │
    ▼
Publish /njc/cmd_vel (TwistStamped, base_link frame)
```

**The pipeline is a direct port of** `~/njc/examples/inference_pipeline_example.py`. Every NJC library call (`DLOProcessor`, `DLOBuffer`, `InferenceModel`, `build_controller`) is used exactly as the example demonstrates — the node just wraps them in ROS subscribers/publishers.

---

## How the Pieces Connect

```
EXISTING NODES (already running)              NEW NODES (njc_ros)
─────────────────────────────────             ─────────────────────

natnet_pub_node ──────────────────┐
  /natnet/unlabeled_marker_data   │
  (Marker POINTS, ~20 markers)    ▼
                              dlo_spline_node
                                  │
                                  ▼ /njc/cable_keypoints (14 ordered kp)
                                  │
natnet_pub_node ──────────────────┤
  /natnet/fixed_ee_pose           │
                                  │
kinova_state_pub_node ────────────┤
  /end_effector_pose              │
                                  │
YOUR CODE (trajectory planner) ───┤
  /njc/target_keypoints           │
                                  ▼
                            njc_controller_node
                                  │
                                  ▼ /njc/cmd_vel (TwistStamped)
                                  │
                            YOUR EEF CONTROLLER (not in this package)
```

**What you still need to provide**:
1. A publisher for `/njc/target_keypoints` — the desired cable shape. This could be a pre-recorded trajectory, a planner, or a simple "hold this shape" static publisher.
2. Something that subscribes to `/njc/cmd_vel` and actually moves the robot. This is typically your MoveIt servo node or a direct Kinova velocity interface.

---

## File Map

```
src/my_code/njc_ros/
├── njc_ros/
│   ├── __init__.py
│   ├── conversions.py          ← ROS msg ↔ NumPy (4 functions)
│   ├── dlo_spline_node.py      ← Marker cloud → ordered keypoints
│   └── njc_controller_node.py  ← Keypoints + poses → velocity command
├── config/
│   └── njc_params.yaml         ← All parameter defaults (edit this to tune)
├── launch/
│   ├── njc_controller.launch.py     ← Launch NJC nodes only (add-on)
│   └── njc_full_system.launch.py    ← Launch everything (bringup + NJC)
├── package.xml
├── setup.py / setup.cfg
└── README.md                   ← Reference docs (params table, topics, troubleshooting)
```

---

## conversions.py — The Glue Layer

Four small functions that bridge ROS messages and NumPy arrays:

| Function | Input | Output | Used by |
|----------|-------|--------|---------|
| `marker_points_to_numpy` | `Marker` (POINTS) | `ndarray (N,3)` | dlo_spline_node |
| `points_array_to_numpy` | `PointsArray` | `ndarray (N,3)` | njc_controller_node |
| `pose_stamped_to_pos_quat` | `PoseStamped` | `(pos[3], quat_xyzw[4])` | njc_controller_node |
| `cmd_to_twist_stamped` | `CMDMsg` | `TwistStamped` | njc_controller_node |

---

## Safety Mechanisms

| Mechanism | Where | What it does |
|-----------|-------|-------------|
| **Staleness check** | dlo_spline_node | Drops frames older than 100ms |
| **Min marker count** | dlo_spline_node | Skips if < 10 markers visible |
| **Spline try/except** | dlo_spline_node | Catches scipy failures gracefully |
| **Watchdog** | njc_controller_node | Stops publishing if ANY input > 200ms old |
| **Buffer warmup** | njc_controller_node | Waits for full context window before first inference |
| **Velocity saturation** | njc_controller_node | Clamps to 0.5 m/s linear, 1.0 rad/s angular |

If any sensor drops out or lags, the controller **stops sending commands** rather than sending stale ones.

---

## How to Run It

```bash
# 1. Make sure pip deps are installed
pip install -e ~/njc
pip install -e ~/dlopredict

# 2. Build
cd ~/data_collection
colcon build --symlink-install --packages-select njc_ros
source install/setup.bash

# 3a. If data_collection_bringup is already running:
ros2 launch njc_ros njc_controller.launch.py model_dir:=/path/to/deploy/dir

# 3b. Or launch everything at once:
ros2 launch njc_ros njc_full_system.launch.py model_dir:=/path/to/deploy/dir
```

The `model_dir` must contain:
- `config.yaml` (model architecture + data config, saved during training)
- `checkpoint_best.pth` (model weights)

---

## Tuning Parameters

All in `config/njc_params.yaml`. The most likely ones you'll want to change:

| Parameter | Default | When to change |
|-----------|---------|----------------|
| `controller_type` | `dls` | Try `lyapunov` near singularities |
| `kp` | 1.0 | Lower for slower, more cautious motion |
| `damping` | 0.1 | Increase for more stable but slower response |
| `max_linear_vel` | 0.5 | Lower for safety during initial tests |
| `min_markers` | 10 | Lower if your cable has fewer markers |
| `num_kp` | 14 | Must match what the model was trained on |
| `control_rate` | 30.0 | Lower if inference is too slow for real-time |

---

## What's NOT in This Package

- **Trajectory generation** — you need to publish `/njc/target_keypoints` yourself
- **Robot velocity interface** — something needs to subscribe to `/njc/cmd_vel` and send it to the Kinova
- **Frame transforms** — the controller outputs velocities in `base_link` frame; if your velocity interface expects a different frame, you'll need a transform
- **DIGIT images** — this controller uses marker-based cable tracking only (`digit_images: []` is passed to the processor)
