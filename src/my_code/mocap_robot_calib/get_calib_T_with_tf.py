import numpy as np
import cv2
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation as R
from typing import Dict, Tuple, List

BagKey = int  # integer nanoseconds

def read_pose_series_ns(bag_path: str, topic: str) -> Dict[BagKey, Tuple[np.ndarray, np.ndarray]]:
    """
    Returns a dict: t_ns -> (p[3], R[3,3])
    where p is translation in the header.frame_id (parent) coordinates and
    R is rotation matrix for parent_T_child (PoseStamped pose of child in parent).
    """
    storage = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, rosbag2_py.ConverterOptions("", ""))

    pose_cls = get_message("geometry_msgs/msg/PoseStamped")
    out: Dict[BagKey, Tuple[np.ndarray, np.ndarray]] = {}

    while reader.has_next():
        top, data, _ = reader.read_next()
        if top != topic:
            continue
        msg = deserialize_message(data, pose_cls)

        t_ns = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
        p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=np.float64)
        q = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                      msg.pose.orientation.z, msg.pose.orientation.w], dtype=np.float64)
        Rmat = R.from_quat(q).as_matrix().astype(np.float64)  # parent_T_child rotation
        out[t_ns] = (p, Rmat)

    return out

def build_calib_inputs(
    natnet_world_T_marker: Dict[BagKey, Tuple[np.ndarray, np.ndarray]],
    base_T_gripper: Dict[BagKey, Tuple[np.ndarray, np.ndarray]],
) -> Tuple[List[np.ndarray], List[np.ndarray], List[np.ndarray], List[np.ndarray]]:
    """
    OpenCV wants:
      R_world2cam, t_world2cam, R_base2gripper, t_base2gripper
    If NatNet gives world_T_marker (parent_T_child), we INVERT to get (world→marker).
    Robot EE topic is already base→gripper (keep as-is).
    """
    # intersect on exact timestamps
    keys = sorted(set(natnet_world_T_marker.keys()) & set(base_T_gripper.keys()))
    if not keys:
        raise RuntimeError("No matching timestamps between NatNet and EE topics.")

    R_world2marker, t_world2marker = [], []
    R_base2gripper, t_base2gripper = [], []

    for k in keys:
        # NatNet: world_T_marker (parent_T_child) 
        p_wm, R_wm = natnet_world_T_marker[k]

        # R_wm = R_wm.T  # world→marker rotation
        # p_wm = -R_wm @ p_wm  # world→marker translation
        R_world2marker.append(R_wm.astype(np.float64))
        t_world2marker.append(p_wm.reshape(3,1).astype(np.float64))  # (3,1)
        

        # Robot: base_T_gripper already base→gripper
        p_bg, R_bg = base_T_gripper[k]
        R_base2gripper.append(R_bg.astype(np.float64))
        t_base2gripper.append(p_bg.reshape(3,1).astype(np.float64))

    return R_world2marker, t_world2marker, R_base2gripper, t_base2gripper

if __name__ == "__main__":
    bag_path = "/home/rotem/data_collection/calib_bag/calib/calib_0.mcap"
    # NatNet rigid body attached to the camera (marker frame), pose in 'world'
    natnet_topic = "/natnet/robot_ee_pose"     # <-- update if different
    # Robot EE pose in base frame (base→gripper)
    ee_topic = "/end_effector_pose"            # <-- update if different

    natnet_dict = read_pose_series_ns(bag_path, natnet_topic)
    ee_dict = read_pose_series_ns(bag_path, ee_topic)

    R_w2c, t_w2c, R_b2g, t_b2g = build_calib_inputs(natnet_dict, ee_dict)
    print(f"Matched pairs: {len(R_w2c)}")

    # get sample data
    # randomly sample n synchronized pose pairs
    n_samples = 20  # set desired number of samples
    total = len(R_w2c)
    if n_samples <= 0 or n_samples >= total:
        print(f"Using all {total} pairs (n_samples={n_samples} out of range).")
    else:
        rng = np.random.default_rng(42)
        idx = np.sort(rng.choice(total, size=n_samples, replace=False))
        R_w2c = [R_w2c[i] for i in idx]
        t_w2c = [t_w2c[i] for i in idx]
        R_b2g = [R_b2g[i] for i in idx]
        t_b2g = [t_b2g[i] for i in idx]
        print(f"Subsampled {n_samples}/{total} matched pairs.")
    # Solve Robot-World–Hand-Eye
    R_base2world, t_base2world, R_gripper2cam, t_gripper2cam = cv2.calibrateRobotWorldHandEye(
        R_world2cam=R_w2c,
        t_world2cam=t_w2c,
        R_base2gripper=R_b2g,
        t_base2gripper=t_b2g,
        method=cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH
    )

    np.set_printoptions(precision=4, suppress=True)
    def print_tf(name, Rm, tm):
        print(f"\n{name} rotation:\n{Rm}")

        print(f"{name} translation: [{tm.ravel()[0]:2f}, {tm.ravel()[1]:2f}, {tm.ravel()[2]:2f}]")
        quat = R.from_matrix(Rm).as_quat()
        print(f"{name} quaternion: [{quat[0]:2f}, {quat[1]:2f}, {quat[2]:2f}, {quat[3]:2f}]")

    print_tf("Base->World",  R_base2world,  t_base2world)
    print_tf("Gripper->Cam", R_gripper2cam, t_gripper2cam)
    
    
    # get world => base
    R_world2base = R_base2world.T
    t_world2base = -R_world2base @ t_base2world
    print_tf("World->Base", R_world2base, t_world2base)
    
    # get marker => gripper
    R_cam2gripper = R_gripper2cam.T
    t_cam2gripper = -R_cam2gripper @ t_gripper2cam
    print_tf("Marker->Gripper", R_cam2gripper, t_cam2gripper)


