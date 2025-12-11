import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class ImuBiasCorrector(Node):
    def __init__(self):
        super().__init__('imu_bias_corrector')
        
        self.declare_parameter('sampling_duration_sec', 5.0)
        self.declare_parameter('sample_rate_hz', 100.0)
        
        sampling_duration_sec = self.get_parameter('sampling_duration_sec').get_parameter_value().double_value
        sample_rate_hz = self.get_parameter('sample_rate_hz').get_parameter_value().double_value
        self.max_bias_samples = int(sampling_duration_sec * sample_rate_hz)
        
        self.gyro_samples = []
        self.accel_samples = []
        self.is_bias_calculated = False
        self.gyro_bias = np.zeros(3)
        self.accel_bias = np.zeros(3)
        
        # Publisher: Output corrected data to /imu/data_corrected
        self.publisher_ = self.create_publisher(Imu, '/imu/data_corrected', 10)
        
        # Subscriber: Listen to the IMU topic defined in the Bridge
        self.subscription = self.create_subscription(
            Imu,
            # ⬇️ تاپیک ورودی از Bridge
            '/zed/zed_node/imu/data_raw', 
            self.imu_callback,
            10)
            
        self.get_logger().info(f"IMU Bias Corrector started. Listening to /zed/zed_node/imu/data_raw...")

    def imu_callback(self, msg):
        if not self.is_bias_calculated:
            self.collect_bias(msg)
            return

        self.publish_corrected(msg)

    def collect_bias(self, msg):
        if len(self.gyro_samples) < self.max_bias_samples:
            # Store angular velocity (gyro) and linear acceleration (accel)
            self.gyro_samples.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            self.accel_samples.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        else:
            self.calculate_bias()
            self.is_bias_calculated = True
            self.get_logger().info("IMU Bias Correction calculated. Starting data correction.")
            self.get_logger().info(f"Gyro Bias: {self.gyro_bias}")
            self.get_logger().info(f"Accel Bias: {self.accel_bias}")

    def calculate_bias(self):
        self.gyro_bias = np.mean(self.gyro_samples, axis=0)
        self.accel_bias = np.mean(self.accel_samples, axis=0)
        # Note: self.accel_bias[2] includes gravity which should be handled by the filter

    def publish_corrected(self, msg):
        corrected_msg = Imu()
        corrected_msg.header = msg.header
        
        # Gyro Correction
        corrected_msg.angular_velocity.x = msg.angular_velocity.x - self.gyro_bias[0]
        corrected_msg.angular_velocity.y = msg.angular_velocity.y - self.gyro_bias[1]
        corrected_msg.angular_velocity.z = msg.angular_velocity.z - self.gyro_bias[2]
        
        # Accel Correction
        corrected_msg.linear_acceleration.x = msg.linear_acceleration.x - self.accel_bias[0]
        corrected_msg.linear_acceleration.y = msg.linear_acceleration.y - self.accel_bias[1]
        
        # Correct the Z-axis static offset, assuming gravity is 9.81
        corrected_msg.linear_acceleration.z = msg.linear_acceleration.z - (self.accel_bias[2] - 9.81)
        
        # Copy covariance data
        corrected_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        corrected_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        corrected_msg.orientation_covariance = msg.orientation_covariance
        
        self.publisher_.publish(corrected_msg)

def main(args=None):
    rclpy.init(args=args)
    imu_bias_corrector = ImuBiasCorrector()
    rclpy.spin(imu_bias_corrector)
    imu_bias_corrector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()