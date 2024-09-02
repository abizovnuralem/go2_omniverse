# Copyright (c) 2024, RoboVerse community
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import asyncio
import time

import numpy as np


from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from go2_interfaces.msg import Go2State
from std_msgs.msg import Header

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField, Imu
from sensor_msgs_py import point_cloud2



from omni.isaac.orbit.sensors import CameraCfg, Camera
from omni.isaac.sensor import LidarRtx
import omni.replicator.core as rep
from scipy.spatial.transform import Rotation
import omni.isaac.orbit.sim as sim_utils



def update_meshes_for_cloud2(position_array, origin, rot):

    q = rot.cpu().numpy()
    rotation = Rotation.from_quat([q[1], q[2], q[3], q[0]])
    rotated_vectors = rotation.apply(position_array)
    rotated_vectors += origin.cpu().numpy()
    rotated_vectors += [0.0, 0.0, 0.4]
    
    return rotated_vectors


def add_rtx_lidar(num_envs, robot_type, debug=False):
    annotator_lst = []
    for i in range(num_envs):
        if robot_type == "g1":
            lidar_sensor = LidarRtx(f'/World/envs/env_{i}/Robot/head_link/lidar_sensor',
                                    rotation_frequency = 200,
                                    pulse_time=1, 
                                    translation=(0.0, 0.0, 0.0),
                                    orientation=(1.0, 0.0, 0.0, 0.0),
                                    config_file_name= "Unitree_L1",
                                )
        
        else:
            lidar_sensor = LidarRtx(f'/World/envs/env_{i}/Robot/base/lidar_sensor',
                                    rotation_frequency = 200,
                                    pulse_time=1, 
                                    translation=(0.0, 0, 0.4),
                                    orientation=(1.0, 0.0, 0.0, 0.0),
                                    config_file_name= "Unitree_L1",
                                    )

        if debug:
            # Create the debug draw pipeline in the post process graph
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

    for i in range(num_envs):
        # publish ros2 info
        base_node.publish_joints(env.env.scene["robot"].data.joint_names, env.env.scene["robot"].data.joint_pos[i], i)
        base_node.publish_odom(env.env.scene["robot"].data.root_state_w[i, :3], env.env.scene["robot"].data.root_state_w[i, 3:7], i)
        base_node.publish_imu(env.env.scene["robot"].data.root_state_w[i, 3:7], env.env.scene["robot"].data.root_lin_vel_b[i, :], env.env.scene["robot"].data.root_ang_vel_b[i, :], i)
        
        
        if robot_type == "go2":
            base_node.publish_robot_state([
                env.env.scene["contact_forces"].data.net_forces_w[i][4][2], 
                env.env.scene["contact_forces"].data.net_forces_w[i][8][2], 
                env.env.scene["contact_forces"].data.net_forces_w[i][14][2], 
                env.env.scene["contact_forces"].data.net_forces_w[i][18][2]
                ], i)

        try:
            if (time.time() - start_time) > 1/20:
                for j in range(num_envs):
                    data = annotator_lst[j].get_data()
                    point_cloud = update_meshes_for_cloud2(
                        data['data'], env.env.scene["robot"].data.root_state_w[j, :3], 
                        env.env.scene["robot"].data.root_state_w[j, 3:7]
                        )
                    base_node.publish_lidar(point_cloud, j)
                start_time = time.time()
        except :
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
            self.go2_state_pub.append(self.create_publisher(Go2State, f'robot{i}/go2_states', qos_profile))
            self.go2_lidar_pub.append(self.create_publisher(PointCloud2, f'robot{i}/point_cloud2', qos_profile))
            self.odom_pub.append(self.create_publisher(Odometry, f'robot{i}/odom', qos_profile))
            self.imu_pub.append(self.create_publisher(Imu, f'robot{i}/imu', qos_profile))
        self.broadcaster= TransformBroadcaster(self, qos=qos_profile)
        
    def publish_joints(self, joint_names_lst, joint_state_lst, robot_num):
        # Create message
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()

        joint_state_names_formated = []
        for joint_name in joint_names_lst:
            joint_state_names_formated.append(f"robot{robot_num}/"+joint_name)

        joint_state_formated = []
        for joint_state_val in joint_state_lst:
            joint_state_formated.append(joint_state_val.item())

        joint_state.name = joint_state_names_formated
        joint_state.position = joint_state_formated
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
        self.broadcaster.sendTransform(odom_trans)

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

        go2_state = Go2State()
        go2_state.foot_force = [
            int(foot_force_lst[0].item()), 
            int(foot_force_lst[1].item()), 
            int(foot_force_lst[2].item()), 
            int(foot_force_lst[3].item())]
        self.go2_state_pub[robot_num].publish(go2_state) 


    def publish_lidar(self, points, robot_num):

        point_cloud = PointCloud2()
        point_cloud.header = Header(frame_id="odom")
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        point_cloud = point_cloud2.create_cloud(point_cloud.header, fields, points)
        self.go2_lidar_pub[robot_num].publish(point_cloud)


    async def run(self):
        while True:
            self.publish_lidar()
            await asyncio.sleep(0.1)