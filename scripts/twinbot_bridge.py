#!/usr/bin/env python3
"""Twinbot bridge — runs on the Jetson payload computer (ROS 2 Humble).

The Unitree Go2 publishes /lowstate on the ethernet subnet (192.168.123.x)
using CycloneDDS. The Isaac Sim PC listens on WiFi (192.168.0.x) using
FastDDS (bundled Jazzy). These two RMWs cannot discover each other, so
this bridge runs two rclpy processes that share data via a multiprocessing
Queue:

  Process A (CycloneDDS, enP8p1s0):
      subscribes /lowstate (unitree_go/msg/LowState, ~500 Hz)
      → puts joint positions on shared Queue

  Process B (FastDDS, wlx*):
      reads Queue → publishes /real_dog/joint_states (sensor_msgs/JointState)
      also forwards /utlidar/robot_odom → /real_dog/odom

Usage (on Jetson):
  source /opt/ros/humble/setup.bash
  source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
  python3 twinbot_bridge.py
"""
import os
import sys
import time
import multiprocessing as mp

# Unitree Go2 SDK motor_state ordering (indices 0-11):
#   0-2: FR hip/thigh/calf   3-5: FL hip/thigh/calf
#   6-8: RR hip/thigh/calf   9-11: RL hip/thigh/calf
MOTOR_JOINT_NAMES = [
    "FR_hip_joint",  "FR_thigh_joint",  "FR_calf_joint",
    "FL_hip_joint",  "FL_thigh_joint",  "FL_calf_joint",
    "RR_hip_joint",  "RR_thigh_joint",  "RR_calf_joint",
    "RL_hip_joint",  "RL_thigh_joint",  "RL_calf_joint",
]

ETH_IFACE = "enP8p1s0"
CYCLONE_URI = (
    f"<CycloneDDS><Domain><General>"
    f"<NetworkInterfaceAddress>{ETH_IFACE}</NetworkInterfaceAddress>"
    f"</General></Domain></CycloneDDS>"
)


def cyclone_reader(queue: mp.Queue):
    """Process A: CycloneDDS on eth, subscribes /lowstate."""
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ["CYCLONEDDS_URI"] = CYCLONE_URI

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from unitree_go.msg import LowState

    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )

    class Reader(Node):
        def __init__(self):
            super().__init__("twinbot_reader")
            self._count = 0
            self.create_subscription(LowState, "/lowstate", self._cb, qos)
            self.get_logger().info(f"CycloneDDS reader on {ETH_IFACE} — listening /lowstate")

        def _cb(self, msg):
            pos = [float(msg.motor_state[i].q)       for i in range(12)]
            vel = [float(msg.motor_state[i].dq)      for i in range(12)]
            eff = [float(msg.motor_state[i].tau_est) for i in range(12)]
            # non-blocking put; drop if publisher is behind
            try:
                queue.put_nowait((pos, vel, eff))
            except Exception:
                pass
            self._count += 1
            if self._count % 1000 == 0:
                self.get_logger().info(f"forwarded {self._count} msgs — FL_hip={pos[3]:.3f} rad")

    rclpy.init()
    node = Reader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def fastdds_publisher(queue: mp.Queue):
    """Process B: FastDDS on WiFi, publishes /real_dog/joint_states."""
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    # Unset CYCLONEDDS_URI so FastDDS picks default interface (WiFi)
    os.environ.pop("CYCLONEDDS_URI", None)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import JointState
    from nav_msgs.msg import Odometry

    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )

    class Publisher(Node):
        def __init__(self):
            super().__init__("twinbot_publisher")
            self._pub_js = self.create_publisher(JointState, "/real_dog/joint_states", qos)
            self._pub_odom = self.create_publisher(Odometry, "/real_dog/odom", 10)
            # forward SLAM odom from eth (it's also CycloneDDS, won't cross — skip for now)
            self.create_timer(0.002, self._drain)  # 500 Hz drain
            self._seq = 0
            self.get_logger().info("FastDDS publisher on WiFi — publishing /real_dog/joint_states")

        def _drain(self):
            # drain all pending items, publish only latest
            latest = None
            while True:
                try:
                    latest = queue.get_nowait()
                except Exception:
                    break
            if latest is None:
                return
            pos, vel, eff = latest
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.header.frame_id = "base_link"
            js.name = MOTOR_JOINT_NAMES
            js.position = pos
            js.velocity = vel
            js.effort = eff
            self._pub_js.publish(js)
            self._seq += 1

    rclpy.init()
    node = Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    q: mp.Queue = mp.Queue(maxsize=5)

    reader = mp.Process(target=cyclone_reader,   args=(q,), daemon=True)
    writer = mp.Process(target=fastdds_publisher, args=(q,), daemon=True)

    reader.start()
    writer.start()
    print(f"[twinbot_bridge] reader PID={reader.pid}  publisher PID={writer.pid}", flush=True)
    print(f"[twinbot_bridge] CycloneDDS on {ETH_IFACE} → FastDDS on WiFi", flush=True)

    try:
        reader.join()
    except KeyboardInterrupt:
        print("\n[twinbot_bridge] shutting down")
    finally:
        reader.terminate()
        writer.terminate()
