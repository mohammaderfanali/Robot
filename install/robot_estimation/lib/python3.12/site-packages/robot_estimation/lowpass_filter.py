import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class LowpassFilter(Node):
    def __init__(self):
        super().__init__('lowpass_filter')
        
        self.declare_parameter('lowpass_alpha', 0.1)
        self.alpha = self.get_parameter('lowpass_alpha').get_parameter_value().double_value
        self.get_logger().info(f"Lowpass Filter started with alpha: {self.alpha:.2f}. Listening to /imu/data_corrected")

        self.gyro_prev = [0.0, 0.0, 0.0]
        self.accel_prev = [0.0, 0.0, 0.0]
        
        self.publisher_ = self.create_publisher(Imu, '/imu/data_filtered', 10)
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_corrected', 
            self.imu_callback,
            10)

    def lowpass_filter_func(self, current_value, previous_filtered_value):
        return self.alpha * current_value + (1.0 - self.alpha) * previous_filtered_value

    def imu_callback(self, msg):
        filtered_msg = Imu()
        filtered_msg.header = msg.header
        
        gyro_current = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        for i in range(3):
            self.gyro_prev[i] = self.lowpass_filter_func(gyro_current[i], self.gyro_prev[i])
        
        filtered_msg.angular_velocity.x, filtered_msg.angular_velocity.y, filtered_msg.angular_velocity.z = self.gyro_prev
        
        accel_current = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        for i in range(3):
            self.accel_prev[i] = self.lowpass_filter_func(accel_current[i], self.accel_prev[i])

        filtered_msg.linear_acceleration.x, filtered_msg.linear_acceleration.y, filtered_msg.linear_acceleration.z = self.accel_prev
        
        filtered_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        filtered_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        filtered_msg.orientation_covariance = msg.orientation_covariance
        
        self.publisher_.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    lowpass_filter = LowpassFilter()
    rclpy.spin(lowpass_filter)
    lowpass_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()