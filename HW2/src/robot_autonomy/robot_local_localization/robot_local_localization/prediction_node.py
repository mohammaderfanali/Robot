import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math
import tf_transformations

class PredictionNode(Node):
    def __init__(self):
        super().__init__('prediction_node')
        self.get_logger().info("Prediction Node has started.")

    
        self.declare_parameter('wheel_radius', 0.1) 
        self.declare_parameter('wheel_separation', 0.45) 
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/wheel_odom/odom') 
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.R = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.L = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.CMD_VEL_TOPIC = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.ODOM_TOPIC = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.ODOM_FRAME = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.BASE_FRAME = self.get_parameter('base_frame').get_parameter_value().string_value

 
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.last_time = self.get_clock().now()

 
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            self.CMD_VEL_TOPIC,
            self.cmd_vel_callback,
            10
        )
        
        self.odom_pub = self.create_publisher(Odometry, self.ODOM_TOPIC, 10)
        
        
        self.timer = self.create_timer(0.05, self.publish_odom) # 20 Hz

        self.current_vx = 0.0
        self.current_wz = 0.0


    def cmd_vel_callback(self, msg):
        """
        دریافت دستورات سرعت و ذخیره آنها
        """
        self.current_vx = msg.linear.x
        self.current_wz = msg.angular.z
        
    def publish_odom(self):
        """
        محاسبه موقعیت جدید ربات (Prediction Update) و انتشار پیام Odometry
        """
        
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt < 0.0001:
            return

        vx = self.current_vx
        wz = self.current_wz
        
   
        if abs(wz) > 1e-4:
            
            R_c = vx / wz
            
     
            dx = R_c * (math.sin(self.theta + wz * dt) - math.sin(self.theta))
            dy = R_c * (-math.cos(self.theta + wz * dt) + math.cos(self.theta))
            dtheta = wz * dt
            
        else:
            dx = vx * dt * math.cos(self.theta)
            dy = vx * dt * math.sin(self.theta)
            dtheta = 0.0

        self.x += dx
        self.y += dy
        self.theta += dtheta
        
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.ODOM_FRAME
        odom.child_frame_id = self.BASE_FRAME
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        quaternion = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]
        
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    prediction_node = PredictionNode()
    rclpy.spin(prediction_node)
    prediction_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()