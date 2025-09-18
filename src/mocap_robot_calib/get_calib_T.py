import numpy as np
import cv2
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from scipy.spatial.transform import Rotation as R
# Function to read data from the bag file and extract NatNet and end-effector poses
def get_data(
    bag_path="calib_bag/calib_bag_0.mcap",
    natnet="/natnet/robot_ee_pose",
    ee="/end_effector_pose",
)-> tuple[list[np.ndarray], list[np.ndarray], list[np.ndarray], list[np.ndarray]]:
    # Prepare reader for MCAP
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    pose_cls = get_message("geometry_msgs/msg/PoseStamped")

    # Accumulators
    natnet_pos, natnet_ori, natnet_t = [], [], []
    ee_pos, ee_ori, ee_t = [], [], []

    # Stream messages
    while reader.has_next():
        topic, data, _ = reader.read_next()  # t is bag time (ns)
        if topic != natnet and topic != ee:
            continue

        msg = deserialize_message(data, pose_cls)
        p = msg.pose.position
        q = msg.pose.orientation
        t = msg.header.stamp.sec \
            + msg.header.stamp.nanosec/1e9  # convert to ns
        if topic == natnet:
            natnet_pos.append((p.x, p.y, p.z))
            natnet_ori.append((q.x, q.y, q.z, q.w))
            natnet_t.append(t)
        else:  # topic_b
            ee_pos.append((p.x, p.y, p.z))
            ee_ori.append((q.x, q.y, q.z, q.w))
            ee_t.append(t)

    natnet_t = np.asarray(natnet_t, dtype=np.float64)
    ee_t = np.asarray(ee_t, dtype=np.float64)
    new_natnet_pos = []
    new_natnet_ori = []
    new_ee_pos = []
    new_ee_ori = []
    counter = 0
    for i,idx in enumerate(natnet_t):
        ee_idx = np.where(ee_t == idx)[0]
        if len(ee_idx) == 0:
            counter += 1
            continue
        ee_idx = ee_idx[0]
        new_natnet_pos.append(np.array(natnet_pos[i]))
        new_natnet_ori.append(natnet_ori[i])
        new_ee_pos.append(np.array(ee_pos[ee_idx]))
        new_ee_ori.append(ee_ori[ee_idx])
    print(f"Discarded {counter} unmatched NatNet poses")
    # Convert to NumPy

    
    # Convert orientations to quaternions
    new_natnet_ori = np.asarray(new_natnet_ori, dtype=np.float64)
    new_ee_ori = np.asarray(new_ee_ori, dtype=np.float64)
    new_natnet_ori = [R.from_quat(o).as_matrix() for o in new_natnet_ori]
    new_ee_ori = [R.from_quat(o).as_matrix() for o in new_ee_ori]

    return (new_natnet_pos, new_natnet_ori, new_ee_pos, new_ee_ori)


if __name__ == "__main__":
    t_natnet, R_natnet, t_ee, R_ee = get_data()
    # print(t_natnet.shape, R_natnet.shape, t_ee.shape, R_ee.shape)
    if len(t_natnet) != len(t_ee):
        raise ValueError("NatNet and end-effector pose counts do not match")
    print(f"Found {len(t_natnet)} matching pairs")

    
    # sample n pairs 
    
    # n = 15
    # indices = np.random.choice(len(t_natnet), n, replace=False)
    # t_natnet = [t_natnet[i] for i in indices]
    # R_natnet = [R_natnet[i] for i in indices]
    # t_ee = [t_ee[i] for i in indices]
    # R_ee = [R_ee[i] for i in indices]
    R_base2world, t_base2world, R_gripper2cam, t_gripper2cam = cv2.calibrateRobotWorldHandEye(
        R_world2cam= R_natnet, t_world2cam= t_natnet,
        R_base2gripper= R_ee, t_base2gripper= t_ee,
        # method=cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH  # or 'Tsai', 'Park', 'Horaud', 'Andreff', Daniilidis.

        method=cv2.CALIB_ROBOT_WORLD_HAND_EYE_LI
    )

    np.set_printoptions(precision=4, suppress=True)

    def print_tf(name, R, t):
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = np.asarray(t).ravel()
        print(f'\n{name} rotation:\n{R}')
        print(f'{name} translation: {np.asarray(t).ravel()}')
        # print(f'{name} homogeneous:\n{T}')

    print_tf('Base->World', R_base2world, t_base2world)
    print_tf('Gripper->Cam', R_gripper2cam, t_gripper2cam)
