from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Iterable

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

if TYPE_CHECKING:
    from geometry_msgs.msg import PoseStamped


SERIAL_ORDER = ["D21118", "D21122", "D21123", "D21124"]


@dataclass(frozen=True)
class BufferedImage:
    stamp_ns: int
    image_chw: np.ndarray


def load_yaml(path: str | Path) -> dict:
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def load_norm_stats(path: str | Path) -> dict[str, np.ndarray]:
    stats = load_yaml(path)
    return {
        "digit_mean": np.asarray(stats["digit_image_stats"]["mean"], dtype=np.float32),
        "digit_std": np.asarray(stats["digit_image_stats"]["std"], dtype=np.float32),
        "ee_mean": np.asarray(stats["ee_position_stats"]["mean"], dtype=np.float32),
        "ee_std": np.asarray(stats["ee_position_stats"]["std"], dtype=np.float32),
        "marker_mean": np.asarray(stats["per_marker_stats"]["mean"], dtype=np.float32),
        "marker_std": np.asarray(stats["per_marker_stats"]["std"], dtype=np.float32),
    }


def ensure_existing_path(path_str: str | None) -> str | None:
    if not path_str:
        return None
    path = Path(path_str).expanduser()
    return str(path) if path.exists() else None


def resolve_model_artifacts(
    checkpoint_path: str,
    config_path: str = "",
    norm_stats_path: str = "",
) -> tuple[str, str, str]:
    ckpt_path = Path(checkpoint_path).expanduser()
    if not ckpt_path.exists():
        raise FileNotFoundError(f"Checkpoint not found: {ckpt_path}")

    resolved_config = Path(config_path).expanduser() if config_path else ckpt_path.with_name("config.yaml")
    if not resolved_config.exists():
        raise FileNotFoundError(f"Config not found: {resolved_config}")

    config = load_yaml(resolved_config)
    resolved_norm = Path(norm_stats_path).expanduser() if norm_stats_path else Path(config["norm_stats_path"]).expanduser()
    if not resolved_norm.exists():
        raise FileNotFoundError(f"Normalization stats not found: {resolved_norm}")

    return str(ckpt_path), str(resolved_config), str(resolved_norm)


def sanitize_config_for_inference(config: dict) -> dict:
    sanitized = dict(config)
    sanitized["model_params"] = dict(config["model_params"])
    sanitized["model_params"]["image_backbone"] = dict(config["model_params"]["image_backbone"])
    pretrained_path = sanitized["model_params"]["image_backbone"].get("pretrained_weights_path")
    if pretrained_path and not Path(pretrained_path).expanduser().exists():
        sanitized["model_params"]["image_backbone"]["pretrained_weights_path"] = None
    return sanitized


def image_msg_to_chw_float(bgr_image: np.ndarray, width: int, height: int) -> np.ndarray:
    resized = cv2.resize(bgr_image, (width, height), interpolation=cv2.INTER_LINEAR)
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    return np.transpose(rgb.astype(np.float32) / 255.0, (2, 0, 1))


def load_blank_reference_images(blank_ref_root: str | None, serial_order: Iterable[str], width: int, height: int) -> dict[str, np.ndarray]:
    if not blank_ref_root:
        return {}

    ref_root = Path(blank_ref_root).expanduser()
    if not ref_root.exists():
        return {}

    refs: dict[str, np.ndarray] = {}
    for serial in serial_order:
        candidate_paths = [
            ref_root / f"{serial}.png",
            ref_root / f"ref_frame_{serial}.png",
        ]
        ref_path = next((path for path in candidate_paths if path.exists()), None)
        if ref_path is None:
            raise FileNotFoundError(
                f"Blank reference missing for {serial} in {ref_root}"
            )
        bgr = cv2.imread(str(ref_path), cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError(f"Failed to read reference image: {ref_path}")
        refs[serial] = image_msg_to_chw_float(bgr, width=width, height=height)
    return refs


def normalize_image_stack(images: np.ndarray, mean: np.ndarray, std: np.ndarray) -> np.ndarray:
    mean_reshaped = mean.reshape(1, 3, 1, 1)
    std_reshaped = std.reshape(1, 3, 1, 1)
    return (images - mean_reshaped) / std_reshaped


def normalize_vector(vector: np.ndarray, mean: np.ndarray, std: np.ndarray) -> np.ndarray:
    return (vector - mean) / std


def denormalize_points(points: np.ndarray, mean: np.ndarray, std: np.ndarray) -> np.ndarray:
    return points * std + mean


def pose_stamped_to_arrays(msg: "PoseStamped") -> tuple[np.ndarray, np.ndarray]:
    position = np.array(
        [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
        dtype=np.float32,
    )
    quat = np.array(
        [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ],
        dtype=np.float32,
    )
    return position, quat


def get_ee_relative_to_fixed(
    ee_position: np.ndarray,
    ee_quat: np.ndarray,
    fixed_position: np.ndarray,
    fixed_quat: np.ndarray,
) -> np.ndarray:
    rot_base_to_ee = Rotation.from_quat(ee_quat).as_matrix()
    rot_base_to_fixed = Rotation.from_quat(fixed_quat).as_matrix()
    rot_fixed_to_base = rot_base_to_fixed.T
    rot_fixed_to_ee = rot_fixed_to_base @ rot_base_to_ee
    trans_fixed_to_ee = rot_fixed_to_base @ (ee_position - fixed_position)
    quat_relative = Rotation.from_matrix(rot_fixed_to_ee).as_quat().astype(np.float32)
    return np.concatenate([trans_fixed_to_ee.astype(np.float32), quat_relative], axis=0)


def transform_points_fixed_to_base(points_fixed: np.ndarray, fixed_position: np.ndarray, fixed_quat: np.ndarray) -> np.ndarray:
    rot_base_to_fixed = Rotation.from_quat(fixed_quat).as_matrix()
    return (rot_base_to_fixed @ points_fixed.T).T + fixed_position


def pick_current_and_past_frame(
    buffer: deque[BufferedImage],
    past_window_steps: int,
    max_time_delta_sec: float,
) -> tuple[np.ndarray, np.ndarray, int]:
    current = buffer[-1]
    past_idx = max(0, len(buffer) - 1 - past_window_steps)
    candidate = buffer[past_idx]
    delta_sec = abs(current.stamp_ns - candidate.stamp_ns) / 1e9
    if delta_sec <= max_time_delta_sec:
        past = candidate
    else:
        past = current
    return current.image_chw, past.image_chw, current.stamp_ns


def build_points_array(points: np.ndarray, stamp, frame_id: str):
    from geometry_msgs.msg import Point
    from my_msgs.msg import PointsArray

    msg = PointsArray()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points]
    return msg


def build_line_strip_marker(
    points: np.ndarray,
    stamp,
    frame_id: str,
    namespace: str,
    marker_id: int,
    rgba: tuple[float, float, float, float],
) -> Marker:
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker

    marker = Marker()
    marker.header.stamp = stamp
    marker.header.frame_id = frame_id
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.005
    marker.color.r = rgba[0]
    marker.color.g = rgba[1]
    marker.color.b = rgba[2]
    marker.color.a = rgba[3]
    marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points]
    return marker
