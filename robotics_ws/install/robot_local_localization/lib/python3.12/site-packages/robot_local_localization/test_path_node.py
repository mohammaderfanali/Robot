#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path # تغییر: Odometry برای اندازه‌گیری
from std_msgs.msg import Float64MultiArray # این دیگر برای اندازه‌گیری استفاده نخواهد شد
import math
import numpy as np
import tf_transformations # برای تبدیل Yaw به Quaternion در مسیر اندازه‌گیری
from geometry_msgs.msg import Quaternion # برای تبدیل Quaternion به Yaw


class TestPathNode(Node):
    def __init__(self):
        super().__init__('test_path_node')
        self.get_logger().info("Test Path Node started, commanding rectangular path.")

        # ==========================================================
        # 1. پارامترهای تاپیک‌ها و مسیر (بدون تغییر)
        # ==========================================================
        self.declare_parameter('ekf_odom_topic', '/ekf/odom')
        self.declare_parameter('prediction_odom_topic', '/wheel_odom/odom')
        self.declare_parameter('ground_truth_odom_topic', '/odom')
        # توجه: این تاپیک اکنون پیام Odometry منتشر می‌کند
        self.declare_parameter('ekf_measurement_topic', '/ekf/measurement') 
        
        self.declare_parameter('path_update_rate', 10.0)
        self.declare_parameter('side_length', 3.0) 
        self.declare_parameter('linear_speed', 0.2) 
        self.declare_parameter('angular_speed', 0.4) 

        self.EKF_ODOM_TOPIC = self.get_parameter('ekf_odom_topic').get_parameter_value().string_value
        self.PRED_ODOM_TOPIC = self.get_parameter('prediction_odom_topic').get_parameter_value().string_value
        self.GT_ODOM_TOPIC = self.get_parameter('ground_truth_odom_topic').get_parameter_value().string_value
        self.EKF_MEASUREMENT_TOPIC = self.get_parameter('ekf_measurement_topic').get_parameter_value().string_value
        
        self.SIDE_LENGTH = self.get_parameter('side_length').get_parameter_value().double_value
        self.LINEAR_SPEED = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.ANGULAR_SPEED = self.get_parameter('angular_speed').get_parameter_value().double_value
        
        # ==========================================================
        # 2. متغیرهای Path برای RViz2 (چهار مسیر) (بدون تغییر)
        # ==========================================================
        self.ekf_path = Path()
        self.ekf_path.header.frame_id = 'odom'
        self.pred_path = Path()
        self.pred_path.header.frame_id = 'odom'
        self.gt_path = Path()
        self.gt_path.header.frame_id = 'odom'
        self.measurement_path = Path()
        self.measurement_path.header.frame_id = 'odom'
        
        # متغیرهای فرماندهی
        self.side_time = self.SIDE_LENGTH / self.LINEAR_SPEED 
        self.turn_time = math.pi / (2 * self.ANGULAR_SPEED)   
        self.state = 'FORWARD'
        self.side_counter = 0  
        self.start_time = self.get_clock().now()

        # ==========================================================
        # 3. پابلیشرها و سابسکرایبرها
        # ==========================================================
        
        # پابلیشر فرمان
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # پابلیشرهای مسیر (Path)
        self.ekf_path_pub = self.create_publisher(Path, '/ekf_path', 10)
        self.pred_path_pub = self.create_publisher(Path, '/prediction_path', 10)
        self.gt_path_pub = self.create_publisher(Path, '/real_path', 10)
        self.measurement_path_pub = self.create_publisher(Path, '/measurement_path', 10)
        
        # سابسکرایبرهای Odometry (EKF, Prediction, Ground Truth)
        self.create_subscription(Odometry, self.EKF_ODOM_TOPIC, self.ekf_odom_callback, 10)
        self.create_subscription(Odometry, self.PRED_ODOM_TOPIC, self.pred_odom_callback, 10)
        self.create_subscription(Odometry, self.GT_ODOM_TOPIC, self.gt_odom_callback, 10)
        
        # !!! اصلاح سابسکرایبر اندازه‌گیری: تغییر از Float64MultiArray به Odometry
        self.create_subscription(Odometry, self.EKF_MEASUREMENT_TOPIC, self.measurement_callback, 10)
        
        # تایمر برای فرماندهی
        self.create_timer(1.0 / self.get_parameter('path_update_rate').get_parameter_value().double_value, self.timer_callback)


    # ==========================================================
    # توابع Callback برای Odometry (بدون تغییر)
    # ==========================================================
    
    @staticmethod
    def quaternion_to_yaw(q: Quaternion) -> float:
        """تبدیل کواترنیون به Yaw (به رادیان)"""
        # q = odom_msg.pose.pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def append_to_path(self, path_msg: Path, odom_msg: Odometry, publisher):
        """تابع کمکی برای تبدیل Odometry به Path و انتشار آن"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'
        pose.pose = odom_msg.pose.pose
        
        path_msg.poses.append(pose)
        publisher.publish(path_msg)
        
    def ekf_odom_callback(self, msg):
        self.append_to_path(self.ekf_path, msg, self.ekf_path_pub)
        
    def pred_odom_callback(self, msg):
        self.append_to_path(self.pred_path, msg, self.pred_path_pub)
        
    def gt_odom_callback(self, msg):
        self.append_to_path(self.gt_path, msg, self.gt_path_pub)
        
    # ==========================================================
    # !!! تابع Callback اصلاح شده برای اندازه‌گیری ترکیبی (Odometry)
    # ==========================================================

    def measurement_callback(self, msg: Odometry): # نوع پیام ورودی را به Odometry تغییر دادیم
        """
        دریافت پیام Odometry اندازه‌گیری ترکیبی (Z) و ترسیم مسیر اندازه‌گیری
        """
        
        # پیام Odometry مستقیماً شامل PoseStamped مورد نیاز Path است.
        pose = PoseStamped()
        
        # 1. کپی کردن هدر و پوز
        pose.header = msg.header
        pose.pose = msg.pose.pose # شامل موقعیت (Position) و جهت (Orientation)

        # 2. اضافه کردن به Path و انتشار
        self.measurement_path.poses.append(pose)
        self.measurement_path_pub.publish(self.measurement_path)

        # (لاگ برای دیباگ)
        x = msg.pose.pose.position.x
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.get_logger().info(f"Measurement Path Updated. X={x:.2f}, Yaw={yaw:.2f}")


    # ==========================================================
    # منطق فرماندهی مستطیلی (Control Logic) (بدون تغییر)
    # ==========================================================

    def timer_callback(self):
     
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9
        
        cmd_msg = Twist()
        
        if self.side_counter >= 4:
            # توقف کامل پس از طی چهار ضلع
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_msg)
            return

        if self.state == 'FORWARD':
            cmd_msg.linear.x = self.LINEAR_SPEED
            cmd_msg.angular.z = 0.0
            
            if elapsed_time >= self.side_time:
                self.start_time = current_time
                self.state = 'TURN_LEFT'
                self.side_counter += 1
                
        elif self.state == 'TURN_LEFT':
            # چرخش ۹۰ درجه
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = self.ANGULAR_SPEED 
            
            if elapsed_time >= self.turn_time:
                self.start_time = current_time
                self.state = 'FORWARD'
        
        self.cmd_vel_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    test_path_node = TestPathNode()
    rclpy.spin(test_path_node)
    test_path_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()