import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler
import math

class ComplementaryFilter(Node):
    def __init__(self):
        super().__init__('complementary_filter')
        
        self.declare_parameter('tau', 0.02)
        self.tau = self.get_parameter('tau').get_parameter_value().double_value
        self.get_logger().info(f"Complementary Filter started with tau: {self.tau:.2f}. Listening to /imu/data_filtered")

        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        
        self.last_time = self.get_clock().now()
        self.is_initialized = False

        self.publisher_ = self.create_publisher(Imu, '/estimation/orientation', 10)
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_filtered', # Input from lowpass filter
            self.imu_callback,
            10)

    def calculate_accel_orientation(self, ax, ay, az):
        # Calculate Roll and Pitch from Accelerometer (gravity reference)
        roll = math.atan2(-ay, az)
        pitch = math.atan2(ax, math.sqrt(ay * ay + az * az))
        return roll, pitch

    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if not self.is_initialized:
            self.current_roll, self.current_pitch = self.calculate_accel_orientation(
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
            )
            self.is_initialized = True
            return

        # 1. Integrate Gyroscope
        self.current_roll += msg.angular_velocity.x * dt
        self.current_pitch += msg.angular_velocity.y * dt
        self.current_yaw += msg.angular_velocity.z * dt 

        # 2. Calculate Accelerometer Orientation
        accel_roll, accel_pitch = self.calculate_accel_orientation(
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        )

        # 3. Complementary Filtering (Fuse Gyro and Accel)
        self.current_roll = (1.0 - self.tau) * self.current_roll + self.tau * accel_roll
        self.current_pitch = (1.0 - self.tau) * self.current_pitch + self.tau * accel_pitch
        
        # Convert RPY to Quaternion
        quat_array = quaternion_from_euler(self.current_roll, self.current_pitch, self.current_yaw)
        
        # Publish the result
        estimated_msg = Imu()
        estimated_msg.header = msg.header
        estimated_msg.orientation.x = quat_array[0]
        estimated_msg.orientation.y = quat_array[1]
        estimated_msg.orientation.z = quat_array[2]
        estimated_msg.orientation.w = quat_array[3]
        
        estimated_msg.orientation_covariance[0] = 0.01 
        estimated_msg.angular_velocity.x = 0.0 
        estimated_msg.linear_acceleration.x = 0.0 

        self.publisher_.publish(estimated_msg)

def main(args=None):
    rclpy.init(args=args)
    complementary_filter = ComplementaryFilter()
    rclpy.spin(complementary_filter)
    complementary_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()