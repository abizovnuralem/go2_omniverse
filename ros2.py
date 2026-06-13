# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-2-Clause

import struct
import time

import numpy as np

from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header, Float32MultiArray

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField, Imu

from isaaclab.sensors import CameraCfg, Camera
from isaacsim.sensors.rtx import LidarRtx
import omni.replicator.core as rep
from scipy.spatial.transform import Rotation
import isaaclab.sim as sim_utils


def _to_numpy(arr):
    """warp.array / torch.Tensor / numpy / list -> numpy.ndarray.

    IsaacLab 4.5 / Isaac Sim 6.0 expose articulation buffers as warp arrays,
    which do not support Python item indexing or iteration; numpy does.
    """
    if hasattr(arr, "numpy"):
        try:
            return arr.numpy()
        except Exception:
            return arr.detach().cpu().numpy()
    return np.asarray(arr)


def update_meshes_for_cloud2(position_array, origin, rot):
    q = rot.cpu().numpy()
    rotation = Rotation.from_quat([q[1], q[2], q[3], q[0]])
    rotated_vectors = rotation.apply(position_array)
    rotated_vectors += origin.cpu().numpy()
    rotated_vectors += [0.0, 0.0, 0.4]
    return rotated_vectors


def _create_point_cloud2(header, points):
    """Build a sensor_msgs/PointCloud2 from an (N,3) float32 array without sensor_msgs_py."""
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] != 3:
        pts = pts.reshape(-1, 3).astype(np.float32)
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = pts.shape[0]
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = pts.tobytes()
    return msg


def add_rtx_lidar(num_envs, robot_type, debug=False):
    annotator_lst = []
    for i in range(num_envs):
        if robot_type == "g1":
            lidar_sensor = LidarRtx(f'/World/envs/env_{i}/Robot/head_link/lidar_sensor',
                                    rotation_frequency=200,
                                    pulse_time=1,
                                    translation=(0.0, 0.0, 0.0),
                                    orientation=(1.0, 0.0, 0.0, 0.0),
                                    config_file_name="Unitree_L1")
        else:
            lidar_sensor = LidarRtx(f'/World/envs/env_{i}/Robot/base/lidar_sensor',
                                    rotation_frequency=200,
                                    pulse_time=1,
                                    translation=(0.0, 0, 0.4),
                                    orientation=(1.0, 0.0, 0.0, 0.0),
                                    config_file_name="Unitree_L1")

        if debug:
            writer = rep.writers.get("RtxLidar" + "DebugDrawPointCloudBuffer")
            writer.attach([lidar_sensor.get_render_product_path()])

        annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacCreateRTXLidarScanBuffer")
        annotator.attach(lidar_sensor.get_render_product_path())
        annotator_lst.append(annotator)
    return annotator_lst


def add_camera(num_envs, robot_type):
    for i in range(num_envs):
        cameraCfg = CameraCfg(
            prim_path=f"/World/envs/env_{i}/Robot/base/front_cam",
            update_period=0.1,
            height=480,
            width=640,
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=24.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 1.0e5)
            ),
            offset=CameraCfg.OffsetCfg(pos=(0.32487, -0.00095, 0.05362), rot=(0.5, -0.5, 0.5, -0.5), convention="ros"),
        )

        if robot_type == "g1":
            cameraCfg.prim_path = f"/World/envs/env_{i}/Robot/head_link/front_cam"
            cameraCfg.offset = CameraCfg.OffsetCfg(pos=(0.0, 0.0, 0.0), rot=(0.5, -0.5, 0.5, -0.5), convention="ros")

        Camera(cameraCfg)


def pub_robo_data_ros2(robot_type, num_envs, base_node, env, annotator_lst, start_time):
    # IsaacLab 4.5 / Isaac Sim 6.0 expose articulation buffers as warp arrays,
    # which do not support Python item indexing. Convert each buffer to numpy
    # once at the source so the publish helpers can index and iterate it.
    robot_data = env.unwrapped.scene["robot"].data
    joint_pos = _to_numpy(robot_data.joint_pos)
    root_state = _to_numpy(robot_data.root_state_w)
    lin_vel_b = _to_numpy(robot_data.root_lin_vel_b)
    ang_vel_b = _to_numpy(robot_data.root_ang_vel_b)
    for i in range(num_envs):
        base_node.publish_joints(robot_data.joint_names, joint_pos[i], i)
        base_node.publish_odom(root_state[i, :3], root_state[i, 3:7], i)
        base_node.publish_imu(root_state[i, 3:7], lin_vel_b[i, :], ang_vel_b[i, :], i)

        if robot_type == "go2":
            net_forces = _to_numpy(env.unwrapped.scene["contact_forces"].data.net_forces_w)
            base_node.publish_robot_state([
                net_forces[i][4][2],
                net_forces[i][8][2],
                net_forces[i][14][2],
                net_forces[i][18][2],
            ], i)

        try:
            if (time.time() - start_time) > 1 / 20:
                for j in range(num_envs):
                    data = annotator_lst[j].get_data()
                    point_cloud = update_meshes_for_cloud2(
                        data['data'], root_state[j, :3], root_state[j, 3:7]
                    )
                    base_node.publish_lidar(point_cloud, j)
                start_time = time.time()
        except Exception:
            pass


class RobotBaseNode(Node):
    def __init__(self, num_envs):
        super().__init__('go2_driver_node')
        qos_profile = QoSProfile(depth=10)

        self.joint_pub = []
        self.go2_state_pub = []
        self.go2_lidar_pub = []
        self.odom_pub = []
        self.imu_pub = []

        for i in range(num_envs):
            self.joint_pub.append(self.create_publisher(JointState, f'robot{i}/joint_states', qos_profile))
            # foot_force published as Float32MultiArray to avoid go2_interfaces dependency
            self.go2_state_pub.append(self.create_publisher(Float32MultiArray, f'robot{i}/foot_force', qos_profile))
            self.go2_lidar_pub.append(self.create_publisher(PointCloud2, f'robot{i}/point_cloud2', qos_profile))
            self.odom_pub.append(self.create_publisher(Odometry, f'robot{i}/odom', qos_profile))
            self.imu_pub.append(self.create_publisher(Imu, f'robot{i}/imu', qos_profile))
        # Publish TF as tf2_msgs/TFMessage on /tf — avoids tf2_ros dependency
        self.tf_pub = self.create_publisher(TFMessage, '/tf', qos_profile)

    def publish_joints(self, joint_names_lst, joint_state_lst, robot_num):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [f"robot{robot_num}/{n}" for n in joint_names_lst]
        joint_state.position = [float(v.item()) for v in joint_state_lst]
        self.joint_pub[robot_num].publish(joint_state)

    def publish_odom(self, base_pos, base_rot, robot_num):
        odom_trans = TransformStamped()
        odom_trans.header.stamp = self.get_clock().now().to_msg()
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = f"robot{robot_num}/base_link"
        odom_trans.transform.translation.x = base_pos[0].item()
        odom_trans.transform.translation.y = base_pos[1].item()
        odom_trans.transform.translation.z = base_pos[2].item()
        odom_trans.transform.rotation.x = base_rot[1].item()
        odom_trans.transform.rotation.y = base_rot[2].item()
        odom_trans.transform.rotation.z = base_rot[3].item()
        odom_trans.transform.rotation.w = base_rot[0].item()
        self.tf_pub.publish(TFMessage(transforms=[odom_trans]))

        odom_topic = Odometry()
        odom_topic.header.stamp = self.get_clock().now().to_msg()
        odom_topic.header.frame_id = "odom"
        odom_topic.child_frame_id = f"robot{robot_num}/base_link"
        odom_topic.pose.pose.position.x = base_pos[0].item()
        odom_topic.pose.pose.position.y = base_pos[1].item()
        odom_topic.pose.pose.position.z = base_pos[2].item()
        odom_topic.pose.pose.orientation.x = base_rot[1].item()
        odom_topic.pose.pose.orientation.y = base_rot[2].item()
        odom_topic.pose.pose.orientation.z = base_rot[3].item()
        odom_topic.pose.pose.orientation.w = base_rot[0].item()
        self.odom_pub[robot_num].publish(odom_topic)

    def publish_imu(self, base_rot, base_lin_vel, base_ang_vel, robot_num):
        imu_trans = Imu()
        imu_trans.header.stamp = self.get_clock().now().to_msg()
        imu_trans.header.frame_id = f"robot{robot_num}/base_link"
        imu_trans.linear_acceleration.x = base_lin_vel[0].item()
        imu_trans.linear_acceleration.y = base_lin_vel[1].item()
        imu_trans.linear_acceleration.z = base_lin_vel[2].item()
        imu_trans.angular_velocity.x = base_ang_vel[0].item()
        imu_trans.angular_velocity.y = base_ang_vel[1].item()
        imu_trans.angular_velocity.z = base_ang_vel[2].item()
        imu_trans.orientation.x = base_rot[1].item()
        imu_trans.orientation.y = base_rot[2].item()
        imu_trans.orientation.z = base_rot[3].item()
        imu_trans.orientation.w = base_rot[0].item()
        self.imu_pub[robot_num].publish(imu_trans)

    def publish_robot_state(self, foot_force_lst, robot_num):
        msg = Float32MultiArray()
        msg.data = [float(v.item()) for v in foot_force_lst]
        self.go2_state_pub[robot_num].publish(msg)

    def publish_lidar(self, points, robot_num):
        header = Header(frame_id="odom")
        header.stamp = self.get_clock().now().to_msg()
        self.go2_lidar_pub[robot_num].publish(_create_point_cloud2(header, points))
