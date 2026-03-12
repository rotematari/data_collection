"""NJC controller node.

Wraps the NJC inference pipeline: subscribes to cable keypoints, EE pose, and
fixed gripper pose, runs DLOProcessor -> DLOBuffer -> InferenceModel ->
Controller, and publishes a TwistStamped velocity command.
"""

from pathlib import Path

import numpy as np
import torch
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, TwistStamped
from my_msgs.msg import PointsArray

from njc.inference import DLOBuffer, DLOProcessor, InferenceModel
from njc.inference.controller import build_controller
from njc.utils.inference import get_device, load_config_from_dir

from njc_ros.conversions import (
    cmd_to_twist_stamped,
    points_array_to_numpy,
    pose_stamped_to_pos_quat,
)


class NJCControllerNode(Node):
    def __init__(self):
        super().__init__('njc_controller_node')

        # ── Declare parameters ──────────────────────────────────────────
        self.declare_parameter('model_dir', '')
        self.declare_parameter('checkpoint_name', 'checkpoint_best.pth')
        self.declare_parameter('controller_type', 'dls')
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('damping', 0.1)
        self.declare_parameter('epsilon', 0.01)
        self.declare_parameter('alpha', 1.0)
        self.declare_parameter('device', 'auto')
        self.declare_parameter('control_rate', 30.0)
        self.declare_parameter('num_kp', 14)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('max_data_age_sec', 0.2)

        # ── Read parameters ─────────────────────────────────────────────
        model_dir = Path(self.get_parameter('model_dir').get_parameter_value().string_value)
        checkpoint_name = self.get_parameter('checkpoint_name').get_parameter_value().string_value
        controller_type = self.get_parameter('controller_type').get_parameter_value().string_value
        kp = self.get_parameter('kp').get_parameter_value().double_value
        damping = self.get_parameter('damping').get_parameter_value().double_value
        epsilon = self.get_parameter('epsilon').get_parameter_value().double_value
        alpha = self.get_parameter('alpha').get_parameter_value().double_value
        device_str = self.get_parameter('device').get_parameter_value().string_value
        control_rate = self.get_parameter('control_rate').get_parameter_value().double_value
        self.num_kp = self.get_parameter('num_kp').get_parameter_value().integer_value
        self.max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.max_data_age_sec = self.get_parameter('max_data_age_sec').get_parameter_value().double_value

        # ── Device ──────────────────────────────────────────────────────
        self.device = get_device(None if device_str == 'auto' else device_str)
        self.get_logger().info(f'Using device: {self.device}')

        # ── Load model config and build pipeline ────────────────────────
        config_dict = load_config_from_dir(model_dir)
        data_config = config_dict.get('data', {})

        self.inf_model = InferenceModel(
            model_dir=model_dir,
            checkpoint_name=checkpoint_name,
            device=str(self.device),
        ).load()

        self.processor = DLOProcessor(config=data_config)

        context_len = 2 if data_config.get('use_prev', False) else 1
        encoder_cfg = config_dict.get('model', {}).get('encoder', {})
        use_geo = encoder_cfg.get('geo_feat_dim', 0) > 0
        self.buffer = DLOBuffer(max_len=context_len, compute_geo=use_geo)
        self.context_len = context_len

        self.controller = build_controller(
            controller_type,
            kp=kp,
            damping=damping,
            epsilon=epsilon,
            alpha=alpha,
        )

        # ── Latest data from subscribers ────────────────────────────────
        self._cable_kp_msg: PointsArray | None = None
        self._ee_pose_msg: PoseStamped | None = None
        self._fixed_pose_msg: PoseStamped | None = None
        self._target_kp_msg: PointsArray | None = None

        # ── Subscribers ─────────────────────────────────────────────────
        self.create_subscription(PointsArray, '/njc/cable_keypoints', self._cable_kp_cb, 10)
        self.create_subscription(PoseStamped, '/end_effector_pose', self._ee_pose_cb, 10)
        self.create_subscription(PoseStamped, '/natnet/fixed_ee_pose', self._fixed_pose_cb, 10)
        self.create_subscription(PointsArray, '/njc/target_keypoints', self._target_kp_cb, 10)

        # ── Publisher ───────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(TwistStamped, '/njc/cmd_vel', 10)

        # ── Timer ───────────────────────────────────────────────────────
        self.create_timer(1.0 / control_rate, self._control_cb)

        self.get_logger().info(
            f'njc_controller_node started: model_dir={model_dir}, '
            f'controller={controller_type}, rate={control_rate} Hz'
        )

    # ── Subscriber callbacks ────────────────────────────────────────────

    def _cable_kp_cb(self, msg: PointsArray):
        self._cable_kp_msg = msg

    def _ee_pose_cb(self, msg: PoseStamped):
        self._ee_pose_msg = msg

    def _fixed_pose_cb(self, msg: PoseStamped):
        self._fixed_pose_msg = msg

    def _target_kp_cb(self, msg: PointsArray):
        self._target_kp_msg = msg

    # ── Control loop ────────────────────────────────────────────────────

    def _control_cb(self):
        # Check all required data is available
        if any(m is None for m in [
            self._cable_kp_msg,
            self._ee_pose_msg,
            self._fixed_pose_msg,
            self._target_kp_msg,
        ]):
            return

        # Watchdog: check data staleness
        now = self.get_clock().now()
        for name, msg in [
            ('cable_kp', self._cable_kp_msg),
            ('ee_pose', self._ee_pose_msg),
            ('fixed_pose', self._fixed_pose_msg),
        ]:
            msg_time = Time.from_msg(msg.header.stamp)
            age_sec = (now - msg_time).nanoseconds * 1e-9
            if age_sec > self.max_data_age_sec:
                self.get_logger().debug(f'Stale data: {name} age={age_sec:.3f}s')
                return

        # Build raw observation
        marker_positions = points_array_to_numpy(self._cable_kp_msg)
        ee_pos, ee_quat = pose_stamped_to_pos_quat(self._ee_pose_msg)
        fixed_pos, fixed_quat = pose_stamped_to_pos_quat(self._fixed_pose_msg)

        raw_sample = {
            'digit_images': [],
            'ee_position': ee_pos,
            'ee_orientation': ee_quat,
            'fixed_position': fixed_pos,
            'fixed_orientation': fixed_quat,
            'marker_positions': marker_positions,
            'timestamp': Time.from_msg(self._cable_kp_msg.header.stamp).nanoseconds * 1e-9,
        }

        # Process -> Buffer -> Infer -> Control
        dlo_msg = self.processor.prepare_dlo_observation(raw_sample)
        self.buffer.append(dlo_msg)

        if len(self.buffer) < self.context_len:
            self.get_logger().info(
                f'Warming up buffer ({len(self.buffer)}/{self.context_len})'
            )
            return

        obs = self.buffer.to_tensor(device=str(self.device))
        obs = {k: v.unsqueeze(0) for k, v in obs.items()}  # add batch dim

        jac_msg = self.inf_model.predict(obs)

        # Current and target cable states
        s_curr = torch.from_numpy(marker_positions).flatten().to(self.device)
        s_target = torch.from_numpy(
            points_array_to_numpy(self._target_kp_msg)
        ).flatten().to(self.device)

        cmd = self.controller.compute_command(jac_msg, s_curr, s_target)

        # Velocity saturation
        linear = np.clip(cmd.linear, -self.max_linear_vel, self.max_linear_vel)
        angular = np.clip(cmd.angular, -self.max_angular_vel, self.max_angular_vel)
        cmd.linear = linear
        cmd.angular = angular

        # Publish
        twist_msg = cmd_to_twist_stamped(
            cmd,
            frame_id='base_link',
            stamp=self.get_clock().now().to_msg(),
        )
        self.cmd_pub.publish(twist_msg)

        # Debug stats
        if hasattr(self.controller, 'last_stats') and self.controller.last_stats:
            stats = self.controller.last_stats
            self.get_logger().debug(
                f'w={stats.manipulability:.4f}, λ={stats.damping_lambda:.4f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = NJCControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
