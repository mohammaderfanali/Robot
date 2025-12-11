#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from sensor_msgs.msg import Imu 
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import math
import tf2_ros 
import tf_transformations


class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        self.get_logger().info("--- EKF Node Started --- (TF Publishing Disabled)")

        # ==========================================================
        # 1. پارامترها
        # ==========================================================
        self.declare_parameter('ekf_odom_topic', '/ekf/odom')
        self.declare_parameter('prediction_topic', '/wheel_odom/odom') 
        self.declare_parameter('measurement_topic', '/ekf/measurement') 
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ekf_rate', 50.0) 

        self.EKF_ODOM_TOPIC = self.get_parameter('ekf_odom_topic').value
        self.PRED_TOPIC = self.get_parameter('prediction_topic').value
        self.MEAS_TOPIC = self.get_parameter('measurement_topic').value
        self.ODOM_FRAME = self.get_parameter('odom_frame').value
        self.BASE_FRAME = self.get_parameter('base_frame').value
        self.EKF_RATE = self.get_parameter('ekf_rate').value

        self.STATE_DIM = 3 
        
        # ==========================================================
        # 2. متغیرهای حالت و کوواریانس EKF
        # ==========================================================
        self.x_hat = np.zeros(self.STATE_DIM) 
        self.P = np.diag([0.1, 0.1, 0.1]) 
        self.Q = np.diag([0.01, 0.01, 0.01]) 
        
        self.last_pred_msg: Odometry = None
        self.last_time = self.get_clock().now()
        
        # ==========================================================
        # 3. پابلیشرها و سابسکرایبرها
        # ==========================================================
        self.ekf_odom_pub = self.create_publisher(Odometry, self.EKF_ODOM_TOPIC, 10)
        
        self.pred_sub = self.create_subscription(
            Odometry, 
            self.PRED_TOPIC, 
            self.prediction_callback, 
            qos_profile_sensor_data
        )
        
        self.meas_sub = self.create_subscription(
            Odometry, 
            self.MEAS_TOPIC, 
            self.measurement_callback, 
            qos_profile_sensor_data
        )
        
        self.timer = self.create_timer(1.0 / self.EKF_RATE, self.timer_callback)
        
        # !!! TF BROADCASTER دیگر استفاده نمی‌شود و می‌تواند کامنت شود !!!
        # self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    # ==========================================================
    # توابع کمکی (بدون تغییر)
    # ==========================================================
    
    @staticmethod
    def quaternion_to_yaw(q: Quaternion) -> float:
        """تبدیل کواترنیون به Yaw (به رادیان)"""
        x, y, z, w = q.x, q.y, q.z, q.w
        return tf_transformations.euler_from_quaternion([x, y, z, w])[2]

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> Quaternion:
        """تبدیل Yaw (به رادیان) به Quaternion"""
        q_array = tf_transformations.quaternion_from_euler(0, 0, yaw)
        q = Quaternion()
        q.x = q_array[0]
        q.y = q_array[1]
        q.z = q_array[2]
        q.w = q_array[3]
        return q

    # ==========================================================
    # 4. مرحله پیش‌بینی (Prediction Step) (بدون تغییر)
    # ==========================================================

    def prediction_callback(self, msg: Odometry):
        
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt < 1e-6 or self.last_pred_msg is None:
            self.last_pred_msg = msg
            self.last_time = current_time
            return
            
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        theta = self.x_hat[2]
        
        if abs(w) < 1e-4:
            dx = v * math.cos(theta) * dt
            dy = v * math.sin(theta) * dt
            dtheta = w * dt
        else:
            dx = (v/w) * (math.sin(theta + w*dt) - math.sin(theta))
            dy = (v/w) * (-math.cos(theta + w*dt) + math.cos(theta))
            dtheta = w * dt

        self.x_hat_minus = self.x_hat.copy()
        self.x_hat_minus[0] += dx
        self.x_hat_minus[1] += dy
        self.x_hat_minus[2] += dtheta
        self.x_hat_minus[2] = math.atan2(math.sin(self.x_hat_minus[2]), math.cos(self.x_hat_minus[2]))

        F = np.eye(self.STATE_DIM)
        if abs(w) < 1e-4:
            F[0, 2] = -v * math.sin(theta) * dt
            F[1, 2] = v * math.cos(theta) * dt
        else:
            F[0, 2] = (v/w) * (math.cos(theta + w*dt) - math.cos(theta))
            F[1, 2] = (v/w) * (math.sin(theta + w*dt) - math.sin(theta))

        self.P_minus = F @ self.P @ F.T + self.Q 
        
        self.x_hat = self.x_hat_minus
        self.P = self.P_minus

        self.last_pred_msg = msg
        self.last_time = current_time
        
    # ==========================================================
    # 5. مرحله به‌روزرسانی (Correction Step) (بدون تغییر)
    # ==========================================================

    def measurement_callback(self, msg: Odometry):
        
        z_x = msg.pose.pose.position.x
        z_y = msg.pose.pose.position.y
        z_theta = self.quaternion_to_yaw(msg.pose.pose.orientation)
        z = np.array([z_x, z_y, z_theta])
        
        R_full = np.array(msg.pose.covariance).reshape((6, 6))
        
        R = np.diag([
            R_full[0, 0], # Variance X
            R_full[1, 1], # Variance Y
            R_full[5, 5]  # Variance Yaw (theta)
        ])
     
        h_x = self.x_hat[0:3]
        H = np.eye(self.STATE_DIM)
        
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        y = z - h_x 
        y[2] = math.atan2(math.sin(y[2]), math.cos(y[2]))

        self.x_hat += K @ y
        self.P = (np.eye(self.STATE_DIM) - K @ H) @ self.P
        
        self.x_hat[2] = math.atan2(math.sin(self.x_hat[2]), math.cos(self.x_hat[2]))

    # ==========================================================
    # 6. تابع انتشار خروجی
    # ==========================================================
    
    def timer_callback(self):
        
        if np.all(self.x_hat == 0):
            return

        current_time = self.get_clock().now().to_msg()
        
        # !!! 1. انتشار تحول TF: این بخش کاملاً حذف شده است !!!
        # ربات دیگر تحول odom -> base_link را منتشر نمی‌کند.
        
        # 2. انتشار پیام Odometry (EKF Result)
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = self.ODOM_FRAME
        odom_msg.child_frame_id = self.BASE_FRAME
        
        q = self.yaw_to_quaternion(self.x_hat[2])

        odom_msg.pose.pose.position.x = self.x_hat[0]
        odom_msg.pose.pose.position.y = self.x_hat[1]
        odom_msg.pose.pose.orientation = q
        
        # انتشار کوواریانس P (تبدیل 3x3 به 6x6 ROS)
        odom_msg.pose.covariance = [0.0] * 36
        
        odom_msg.pose.covariance[0] = self.P[0, 0]  # X
        odom_msg.pose.covariance[7] = self.P[1, 1]  # Y
        odom_msg.pose.covariance[35] = self.P[2, 2] # Yaw
        
        odom_msg.pose.covariance[1] = self.P[0, 1] # X-Y
        odom_msg.pose.covariance[6] = self.P[1, 0] # Y-X
        
        if self.last_pred_msg is not None:
             odom_msg.twist = self.last_pred_msg.twist
        
        self.ekf_odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    ekf_node = EKFNode()
    rclpy.spin(ekf_node)
    ekf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()