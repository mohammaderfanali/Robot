import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math

WHEEL_RADIUS = 0.05 
BASE_WIDTH = 0.40    

class DiffDriveController(Node):
    def __init__(self):
        super().__init__('diff_drive_controller')
        self.right_rpm_publisher_ = self.create_publisher(Float64, 'robot/rpm_right', 10)
        self.left_rpm_publisher_ = self.create_publisher(Float64, 'robot/rpm_left', 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info('Diff Drive Controller started.')

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        omega = msg.angular.z

        v_right = v + (omega * BASE_WIDTH / 2.0)
        v_left = v - (omega * BASE_WIDTH / 2.0)

        if WHEEL_RADIUS == 0.0: return
            
        right_rpm = v_right / (2 * math.pi * WHEEL_RADIUS) * 60
        left_rpm = v_left / (2 * math.pi * WHEEL_RADIUS) * 60

        self.right_rpm_publisher_.publish(Float64(data=right_rpm))
        self.left_rpm_publisher_.publish(Float64(data=left_rpm))

def main(args=None):
    rclpy.init(args=args)
    diff_drive_controller = DiffDriveController()
    rclpy.spin(diff_drive_controller)
    diff_drive_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()