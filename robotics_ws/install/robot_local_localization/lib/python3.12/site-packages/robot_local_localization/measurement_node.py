#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import numpy as np
import math
from typing import Optional
from rclpy.qos import qos_profile_sensor_data 

class MeasurementIMUIntegratedNode(Node):
    def __init__(self):
        super().__init__('measurement_node')
        self.get_logger().info("--- Measurement Node Started: Fused Odometry Mode ---")

        self._init_parameters()
        self._init_state()
        self._init_interfaces()
        
        self.get_logger().info(f"Subscribing to VO: {self.vo_topic}")
        self.get_logger().info(f"Subscribing to IMU: {self.imu_topic}")

    # ---------- Init Methods ----------

    def _init_parameters(self) -> None:
        self.declare_parameter("imu_topic", "/zed/zed_node/imu/data_raw")
        self.declare_parameter("vo_topic", "/vo/odom") 
        self.declare_parameter("measurement_topic", "/ekf/measurement")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("imu_yaw_stddev", 0.05) 

        self.imu_topic = self.get_parameter("imu_topic").value
        self.vo_topic = self.get_parameter("vo_topic").value
        self.measurement_topic = self.get_parameter("measurement_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.imu_yaw_stddev = float(self.get_parameter("imu_yaw_stddev").value)
        self.imu_yaw_var = self.imu_yaw_stddev ** 2
        
        self.vo_xy_var_default = 0.05 ** 2
        self.vo_z_var_default = 0.01 ** 2


    def _init_state(self) -> None:
        self.last_imu_yaw: Optional[float] = None
        self.imu_yaw_var_current: float = self.imu_yaw_var

    def _init_interfaces(self) -> None:
        
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            qos_profile_sensor_data,
        )

        self.vo_sub = self.create_subscription(
            Odometry,
            self.vo_topic,
            self.vo_callback,
            qos_profile_sensor_data,
        )

        self.meas_pub = self.create_publisher(Odometry, self.measurement_topic, 10)


    # ---------- Callbacks ----------

    def imu_callback(self, msg: Imu) -> None:
        q = msg.orientation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.last_imu_yaw = yaw

        if len(msg.orientation_covariance) == 9 and msg.orientation_covariance[8] > 0.0:
            self.imu_yaw_var_current = float(msg.orientation_covariance[8])
        else:
             self.imu_yaw_var_current = self.imu_yaw_var
        
        if self.last_imu_yaw is not None and not hasattr(self, '_imu_connected_log'):
             self.get_logger().info("✅ IMU Connected and receiving data.")
             self._imu_connected_log = True


    def vo_callback(self, msg: Odometry) -> None:
   
        if self.last_imu_yaw is None:
            return

        meas = Odometry()
        meas.header = msg.header
        meas.header.frame_id = self.odom_frame
        meas.child_frame_id = self.base_frame

        meas.pose.pose.position = msg.pose.pose.position

        q = self.yaw_to_quaternion(self.last_imu_yaw)
        meas.pose.pose.orientation = q

        meas.pose.covariance = [0.0] * 36
        vo_cov = msg.pose.covariance

        if len(vo_cov) == 36 and vo_cov[0] > 0.0:
            meas.pose.covariance[0] = vo_cov[0]    # Variance X
            meas.pose.covariance[7] = vo_cov[7]    # Variance Y
            meas.pose.covariance[14] = vo_cov[14]  # Variance Z
        else:
            meas.pose.covariance[0] = self.vo_xy_var_default
            meas.pose.covariance[7] = self.vo_xy_var_default
            meas.pose.covariance[14] = self.vo_z_var_default
        
        meas.pose.covariance[35] = self.imu_yaw_var_current

        meas.twist = msg.twist

        self.meas_pub.publish(meas)



    @staticmethod
    def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        half = 0.5 * yaw
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(half)
        q.w = math.cos(half)
        return q


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MeasurementIMUIntegratedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by Keyboard Interrupt.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()