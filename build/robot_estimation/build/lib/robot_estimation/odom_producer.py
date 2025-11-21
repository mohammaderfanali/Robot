import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import math

WHEEL_RADIUS = 0.05
BASE_WIDTH = 0.40

class OdomProducer(Node):
    def __init__(self):
        super().__init__('odom_producer')
        self.odom_raw_publisher_ = self.create_publisher(Odometry, 'wheel_encoder/odom_raw', 10)
        
        self.create_subscription(Float64, 'robot/rpm_right', self.right_wheel_callback, 10)
        self.create_subscription(Float64, 'robot/rpm_left', self.left_wheel_callback, 10)
        
        self.right_rpm_ = 0.0
        self.left_rpm_ = 0.0
        self.create_timer(0.02, self.update_and_publish_odom)

    def right_wheel_callback(self, msg): self.right_rpm_ = msg.data
    def left_wheel_callback(self, msg): self.left_rpm_ = msg.data

    def update_and_publish_odom(self):
        current_time = self.get_clock().now()
        v_right = self.right_rpm_ * (2 * math.pi / 60) * WHEEL_RADIUS
        v_left = self.left_rpm_ * (2 * math.pi / 60) * WHEEL_RADIUS
        
        v = (v_right + v_left) / 2.0
        omega = (v_right - v_left) / BASE_WIDTH
        
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link' 
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        self.odom_raw_publisher_.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    odom_producer = OdomProducer()
    rclpy.spin(odom_producer)
    odom_producer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()