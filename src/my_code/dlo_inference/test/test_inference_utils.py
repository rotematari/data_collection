from collections import deque

import numpy as np

from dlo_inference.inference_utils import (
    BufferedImage,
    get_ee_relative_to_fixed,
    pick_current_and_past_frame,
    transform_points_fixed_to_base,
)


def test_relative_pose_is_identity_when_frames_match():
    ee_pos = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    ee_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
    fixed_pos = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    fixed_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)

    relative = get_ee_relative_to_fixed(ee_pos, ee_quat, fixed_pos, fixed_quat)

    assert np.allclose(relative[:3], np.zeros(3, dtype=np.float32))
    assert np.allclose(relative[3:], ee_quat)


def test_transform_points_fixed_to_base_applies_translation():
    points_fixed = np.array([[0.0, 0.0, 0.0], [1.0, -1.0, 2.0]], dtype=np.float32)
    fixed_pos = np.array([0.5, 1.5, -2.0], dtype=np.float32)
    fixed_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)

    points_base = transform_points_fixed_to_base(points_fixed, fixed_pos, fixed_quat)

    assert np.allclose(points_base, points_fixed + fixed_pos)


def test_pick_current_and_past_frame_uses_current_when_delta_is_too_large():
    image = np.zeros((3, 2, 2), dtype=np.float32)
    buffer = deque(
        [
            BufferedImage(stamp_ns=0, image_chw=image + 1.0),
            BufferedImage(stamp_ns=int(2e9), image_chw=image + 2.0),
        ]
    )

    current, past, stamp_ns = pick_current_and_past_frame(
        buffer, past_window_steps=1, max_time_delta_sec=0.1
    )

    assert stamp_ns == int(2e9)
    assert np.allclose(current, image + 2.0)
    assert np.allclose(past, image + 2.0)
