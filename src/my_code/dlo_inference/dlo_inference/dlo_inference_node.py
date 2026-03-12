from __future__ import annotations

import sys
import threading
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
import torch
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from my_msgs.msg import PointsArray
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker

from dlo_inference.inference_utils import (
    SERIAL_ORDER,
    BufferedImage,
    build_line_strip_marker,
    build_points_array,
    denormalize_points,
    ensure_existing_path,
    get_ee_relative_to_fixed,
    image_msg_to_chw_float,
    load_blank_reference_images,
    load_norm_stats,
    load_yaml,
    normalize_image_stack,
    normalize_vector,
    pick_current_and_past_frame,
    pose_stamped_to_arrays,
    resolve_model_artifacts,
    sanitize_config_for_inference,
    transform_points_fixed_to_base,
)


class DLOInferenceNode(Node):
    def __init__(self) -> None:
        super().__init__("dlo_inference_node")
        self._declare_parameters()
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.serial_order = list(
            self.get_parameter("digit_serials").get_parameter_value().string_array_value
        ) or list(SERIAL_ORDER)
        self.image_buffers = {
            serial: deque(maxlen=self.buffer_size) for serial in self.serial_order
        }
        self.reference_images: dict[str, np.ndarray] = {}
        self.latest_ee_pose: PoseStamped | None = None
        self.latest_fixed_pose: PoseStamped | None = None

        self.device = self._resolve_device()
        self._load_model_and_artifacts()
        self._setup_ros_io()

        self.get_logger().info(
            f"Loaded checkpoint: {self.checkpoint_path} on device={self.device.type}"
        )

    def _declare_parameters(self) -> None:
        default_ckpt_dir = (
            "/home/rotem/dlopredict/results/best_runs/"
            "09_02_20_05_np32_sinencTrue_attTrue_normTrue_norminpTrue_poseFalse_"
            "diffTrue_blankrefTrue_img320x240_bs32_lr0.001_wd0.0001_dp0.1_ep10"
        )
        self.declare_parameter("dlopredict_root", "/home/rotem/dlopredict")
        self.declare_parameter("checkpoint_path", f"{default_ckpt_dir}/best.pt")
        self.declare_parameter("config_path", f"{default_ckpt_dir}/config.yaml")
        self.declare_parameter(
            "norm_stats_path", "/home/rotem/dlopredict/src/simplepredict/configs/norm_stats.yaml"
        )
        self.declare_parameter("blank_ref_root", "")
        self.declare_parameter("device", "auto")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("buffer_size", 8)
        self.declare_parameter("max_sync_slop_sec", 0.08)
        self.declare_parameter("fixed_prediction_frame_id", "fixed_ee")
        self.declare_parameter("digit_serials", SERIAL_ORDER)
        self.declare_parameter("ee_pose_topic", "/end_effector_pose")
        self.declare_parameter("fixed_ee_pose_topic", "/natnet/fixed_ee_pose")
        self.declare_parameter("publish_base_frame", True)
        self.declare_parameter("publish_fixed_frame", True)

        self.buffer_size = max(
            2, self.get_parameter("buffer_size").get_parameter_value().integer_value
        )
        self.publish_rate = (
            self.get_parameter("publish_rate").get_parameter_value().double_value
        )
        self.max_sync_slop_sec = (
            self.get_parameter("max_sync_slop_sec").get_parameter_value().double_value
        )

    def _resolve_device(self) -> torch.device:
        requested = self.get_parameter("device").get_parameter_value().string_value.lower()
        if requested == "cuda":
            return torch.device("cuda")
        if requested == "cpu":
            return torch.device("cpu")
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")

    def _load_model_and_artifacts(self) -> None:
        self.checkpoint_path, self.config_path, self.norm_stats_path = resolve_model_artifacts(
            self.get_parameter("checkpoint_path").get_parameter_value().string_value,
            self.get_parameter("config_path").get_parameter_value().string_value,
            self.get_parameter("norm_stats_path").get_parameter_value().string_value,
        )

        config = load_yaml(self.config_path)
        blank_ref_override = ensure_existing_path(
            self.get_parameter("blank_ref_root").get_parameter_value().string_value
        )
        if blank_ref_override:
            config["blank_ref_root"] = blank_ref_override

        self.config = sanitize_config_for_inference(config)
        self.width = int(self.config["img_size_W"])
        self.height = int(self.config["img_size_H"])
        self.past_window_steps = int(self.config.get("past_window_steps", 1))
        self.max_time_delta = float(self.config.get("max_time_delta", 0.13))
        self.normalize_inputs = bool(self.config.get("normalize_inputs", True))
        self.normalize_outputs = bool(self.config.get("normalize", True))
        self.use_diff_images = bool(self.config.get("use_diff_images", False))
        self.use_blank_ref = bool(self.config.get("use_blank_ref", False))
        self.only_pose = bool(self.config.get("only_pose", False))

        self.stats = load_norm_stats(self.norm_stats_path)
        blank_ref_root = self._resolve_blank_ref_root()
        self.blank_refs = load_blank_reference_images(
            blank_ref_root,
            self.serial_order,
            width=self.width,
            height=self.height,
        )
        if self.use_blank_ref and not self.blank_refs:
            self.get_logger().warn(
                "Configured to use blank reference images from assets, but none were found. "
                "Falling back to live /ref_image topics."
            )
            self.use_blank_ref = False

        dlopredict_root = Path(
            self.get_parameter("dlopredict_root").get_parameter_value().string_value
        ).expanduser()
        src_root = dlopredict_root / "src"
        for candidate in (str(dlopredict_root), str(src_root)):
            if candidate not in sys.path:
                sys.path.insert(0, candidate)

        from simplepredict.models.sparsh_enc import DLOStateModel

        checkpoint = torch.load(self.checkpoint_path, map_location="cpu", weights_only=False)
        self.model = DLOStateModel(self.config).to(self.device)
        self.model.load_state_dict(checkpoint["state_dict"], strict=True)
        self.model.eval()

    def _resolve_blank_ref_root(self) -> str | None:
        if not self.use_blank_ref:
            return ensure_existing_path(self.config.get("blank_ref_root"))

        try:
            digit_pub_share = Path(get_package_share_directory("digit_pub"))
            asset_root = digit_pub_share / "assets" / "ref_frames"
            if asset_root.exists():
                return str(asset_root)
        except PackageNotFoundError:
            pass

        source_asset_root = Path(__file__).resolve().parents[2] / "digit_pub" / "assets" / "ref_frames"
        if source_asset_root.exists():
            return str(source_asset_root)

        return None

    def _setup_ros_io(self) -> None:
        self.sub_cb_group = ReentrantCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        ref_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        for serial in self.serial_order:
            self.create_subscription(
                Image,
                f"/digit/{serial}/image_raw",
                self._make_image_callback(serial),
                image_qos,
                callback_group=self.sub_cb_group,
            )
            if not self.use_blank_ref:
                self.create_subscription(
                    Image,
                    f"/digit/{serial}/ref_image",
                    self._make_ref_callback(serial),
                    ref_qos,
                    callback_group=self.sub_cb_group,
                )

        self.create_subscription(
            PoseStamped,
            self.get_parameter("ee_pose_topic").get_parameter_value().string_value,
            self._ee_pose_callback,
            image_qos,
            callback_group=self.sub_cb_group,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("fixed_ee_pose_topic").get_parameter_value().string_value,
            self._fixed_pose_callback,
            image_qos,
            callback_group=self.sub_cb_group,
        )

        self.fixed_points_pub = self.create_publisher(
            PointsArray, "/dlo_inference/prediction_fixed", 10
        )
        self.base_points_pub = self.create_publisher(
            PointsArray, "/dlo_inference/prediction_base", 10
        )
        self.fixed_marker_pub = self.create_publisher(
            Marker, "/dlo_inference/prediction_fixed_marker", 10
        )
        self.base_marker_pub = self.create_publisher(
            Marker, "/dlo_inference/prediction_base_marker", 10
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self._timer_callback,
            callback_group=self.timer_cb_group,
        )

    def _make_image_callback(self, serial: str):
        def callback(msg: Image) -> None:
            stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            chw = image_msg_to_chw_float(bgr, width=self.width, height=self.height)
            with self.lock:
                self.image_buffers[serial].append(
                    BufferedImage(stamp_ns=stamp_ns, image_chw=chw)
                )

        return callback

    def _make_ref_callback(self, serial: str):
        def callback(msg: Image) -> None:
            if self.use_blank_ref:
                return
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            chw = image_msg_to_chw_float(bgr, width=self.width, height=self.height)
            with self.lock:
                self.reference_images[serial] = chw

        return callback

    def _ee_pose_callback(self, msg: PoseStamped) -> None:
        with self.lock:
            self.latest_ee_pose = msg

    def _fixed_pose_callback(self, msg: PoseStamped) -> None:
        with self.lock:
            self.latest_fixed_pose = msg

    def _timer_callback(self) -> None:
        bundle = self._build_model_inputs()
        if bundle is None:
            return

        imgs, ee_pose, fixed_pose_msg, stamp_ns = bundle
        stamp_msg = Time(nanoseconds=stamp_ns).to_msg()

        with torch.inference_mode():
            imgs_tensor = None if self.only_pose else torch.from_numpy(imgs).unsqueeze(0).to(self.device)
            pose_tensor = torch.from_numpy(ee_pose).unsqueeze(0).to(self.device)
            with torch.autocast(device_type="cuda", enabled=(self.device.type == "cuda")):
                pred = self.model(pose_tensor, imgs_tensor)

        pred_np = pred.squeeze(0).detach().cpu().float().numpy()
        if self.normalize_outputs:
            pred_np = denormalize_points(
                pred_np, self.stats["marker_mean"], self.stats["marker_std"]
            )

        fixed_position, fixed_quat = pose_stamped_to_arrays(fixed_pose_msg)
        base_points = transform_points_fixed_to_base(pred_np, fixed_position, fixed_quat)

        if self.get_parameter("publish_fixed_frame").get_parameter_value().bool_value:
            frame_id = self.get_parameter(
                "fixed_prediction_frame_id"
            ).get_parameter_value().string_value
            self.fixed_points_pub.publish(build_points_array(pred_np, stamp_msg, frame_id))
            self.fixed_marker_pub.publish(
                build_line_strip_marker(
                    pred_np,
                    stamp_msg,
                    frame_id,
                    namespace="dlo_prediction_fixed",
                    marker_id=0,
                    rgba=(1.0, 0.0, 1.0, 1.0),
                )
            )

        if self.get_parameter("publish_base_frame").get_parameter_value().bool_value:
            frame_id = fixed_pose_msg.header.frame_id or "base_link"
            self.base_points_pub.publish(build_points_array(base_points, stamp_msg, frame_id))
            self.base_marker_pub.publish(
                build_line_strip_marker(
                    base_points,
                    stamp_msg,
                    frame_id,
                    namespace="dlo_prediction_base",
                    marker_id=1,
                    rgba=(0.0, 1.0, 1.0, 1.0),
                )
            )

    def _build_model_inputs(self):
        with self.lock:
            if self.latest_ee_pose is None or self.latest_fixed_pose is None:
                return None
            if any(len(self.image_buffers[serial]) == 0 for serial in self.serial_order):
                return None
            if self.use_diff_images and not self.use_blank_ref:
                if any(serial not in self.reference_images for serial in self.serial_order):
                    return None

            current_images = []
            past_images = []
            current_stamps = []
            for serial in self.serial_order:
                curr, past, stamp_ns = pick_current_and_past_frame(
                    self.image_buffers[serial],
                    past_window_steps=self.past_window_steps,
                    max_time_delta_sec=self.max_time_delta,
                )
                current_images.append(curr)
                past_images.append(past)
                current_stamps.append(stamp_ns)

            ee_pose_msg = self.latest_ee_pose
            fixed_pose_msg = self.latest_fixed_pose

        image_stamp_ns = min(current_stamps)
        max_image_skew_sec = (max(current_stamps) - min(current_stamps)) / 1e9
        if max_image_skew_sec > self.max_sync_slop_sec:
            return None

        ee_stamp_ns = Time.from_msg(ee_pose_msg.header.stamp).nanoseconds
        fixed_stamp_ns = Time.from_msg(fixed_pose_msg.header.stamp).nanoseconds
        if abs(ee_stamp_ns - image_stamp_ns) / 1e9 > self.max_sync_slop_sec:
            return None
        if abs(fixed_stamp_ns - image_stamp_ns) / 1e9 > self.max_sync_slop_sec:
            return None

        imgs_curr = np.stack(current_images, axis=0).astype(np.float32)
        imgs_prev = np.stack(past_images, axis=0).astype(np.float32)

        if self.use_diff_images:
            if self.use_blank_ref:
                refs = np.stack([self.blank_refs[serial] for serial in self.serial_order], axis=0)
            else:
                refs = np.stack(
                    [self.reference_images[serial] for serial in self.serial_order], axis=0
                )
        else:
            refs = None

        if self.normalize_inputs:
            imgs_curr = normalize_image_stack(
                imgs_curr, self.stats["digit_mean"], self.stats["digit_std"]
            )
            imgs_prev = normalize_image_stack(
                imgs_prev, self.stats["digit_mean"], self.stats["digit_std"]
            )
            if refs is not None:
                refs = normalize_image_stack(
                    refs, self.stats["digit_mean"], self.stats["digit_std"]
                )

        if refs is not None:
            imgs_curr = imgs_curr - refs
            imgs_prev = imgs_prev - refs

        imgs_combined = np.concatenate([imgs_curr, imgs_prev], axis=1).astype(np.float32)

        ee_position, ee_quat = pose_stamped_to_arrays(ee_pose_msg)
        fixed_position, fixed_quat = pose_stamped_to_arrays(fixed_pose_msg)
        ee_pose = get_ee_relative_to_fixed(
            ee_position, ee_quat, fixed_position, fixed_quat
        ).astype(np.float32)
        if self.normalize_inputs:
            ee_pose = normalize_vector(
                ee_pose, self.stats["ee_mean"], self.stats["ee_std"]
            ).astype(np.float32)

        return imgs_combined, ee_pose, fixed_pose_msg, image_stamp_ns


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DLOInferenceNode()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
