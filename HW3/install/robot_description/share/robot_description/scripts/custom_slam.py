#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import tf_transformations
import numpy as np
import math
import os

class CustomSLAM(Node):
    def __init__(self):
        super().__init__('custom_slam')

        # --- تنظیمات نقشه ---
        self.resolution = 0.05  # هر پیکسل 5 سانتی‌متر
        self.width = 800        # عرض نقشه (پیکسل) -> 40 متر
        self.height = 800       # طول نقشه (پیکسل) -> 40 متر
        self.origin_x = -20.0   # نقطه شروع نقشه در دنیای واقعی
        self.origin_y = -20.0
        
        # ماتریس نقشه: -1 (ناشناخته)، 0 (خالی)، 100 (دیوار)
        self.map_grid = np.full((self.height, self.width), -1, dtype=np.int8)

        # متغیرهای وضعیت
        self.last_pose = None
        self.initialized = False

        # --- Subscriber ها ---
        self.create_subscription(Odometry, '/ekf_diff_imu/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # --- Publisher ---
     # --- تنظیمات QoS مخصوص نقشه (تا RViz قهر نکند!) ---
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # --- Publisher ---
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)

        # --- Service برای ذخیره نقشه ---
        self.create_service(Empty, '/save_map', self.save_map_callback)

        self.get_logger().info("Custom SLAM Node Started! Mapping the environment...")
        self.get_logger().info("To save the map, run: ros2 service call /save_map std_srvs/srv/Empty")

    def world_to_grid(self, x, y):
        """ تبدیل مختصات واقعی به پیکسل‌های نقشه """
        mx = int((x - self.origin_x) / self.resolution)
        my = int((y - self.origin_y) / self.resolution)
        return mx, my

    def bresenham_line(self, x0, y0, x1, y1):
        """ الگوریتم برِسنِهام برای پیدا کردن پیکسل‌های روی خط لیزر """
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return points

    def odom_callback(self, msg):
        """ دریافت مکان فعلی ربات """
        q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.last_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.initialized = True

    def scan_callback(self, msg):
        """ دریافت لیزر، کشف محیط و آپدیت نقشه """
        if not self.initialized or self.last_pose is None:
            return

        rx, ry, ryaw = self.last_pose
        start_mx, start_my = self.world_to_grid(rx, ry)

        # برای جلوگیری از هنگ کردن، از هر 5 اشعه لیزر یکی را پردازش می‌کنیم
        step = 5
        ranges = msg.ranges[::step]
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment * step)

        map_updated = False

        for r, ang in zip(ranges, angles):
            if r < msg.range_min or r > msg.range_max or math.isinf(r) or math.isnan(r):
                continue

            # نقطه برخورد لیزر در دنیای واقعی
            hit_x = rx + r * math.cos(ryaw + ang)
            hit_y = ry + r * math.sin(ryaw + ang)

            # نقطه برخورد لیزر در نقشه
            end_mx, end_my = self.world_to_grid(hit_x, hit_y)

            # رسم خط از ربات تا نقطه برخورد (فضای خالی)
            line_points = self.bresenham_line(start_mx, start_my, end_mx, end_my)
            
            for i, (px, py) in enumerate(line_points):
                if 0 <= px < self.width and 0 <= py < self.height:
                    # نقطه آخر دیوار است (100)، بقیه مسیر فضای خالی است (0)
                    if i == len(line_points) - 1:
                        self.map_grid[py, px] = 100
                    else:
                        # اگر قبلاً دیوار نبوده، خالی‌اش کن
                        if self.map_grid[py, px] != 100:
                            self.map_grid[py, px] = 0
                    map_updated = True

        if map_updated:
            self.publish_map()

    def publish_map(self):
        """ تبدیل ماتریس به فرمت ROS و انتشار آن """
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0

        # تبدیل آرایه دو بعدی به یک بعدی برای ارسال
        msg.data = self.map_grid.flatten().tolist()
        self.map_pub.publish(msg)

    def save_map_callback(self, request, response):
        """ تابع ذخیره نقشه به صورت pgm و yaml (بخش 4 سوال) """
        self.get_logger().info("Saving map to disk...")
        
        map_name = "my_custom_map"
        pgm_file = f"{map_name}.pgm"
        yaml_file = f"{map_name}.yaml"

        # 1. ساخت فایل PGM
        # در PGM: 255 یعنی خالی (سفید)، 0 یعنی دیوار (سیاه)، 205 یعنی ناشناخته (خاکستری)
        pgm_data = bytearray()
        for val in reversed(self.map_grid):  # ROS نقشه‌ها را از پایین به بالا می‌خواند
            for cell in val:
                if cell == 100:
                    pgm_data.append(0)    # دیوار -> سیاه
                elif cell == 0:
                    pgm_data.append(255)  # خالی -> سفید
                else:
                    pgm_data.append(205)  # ناشناخته -> خاکستری

        with open(pgm_file, 'wb') as f:
            f.write(f"P5\n{self.width} {self.height}\n255\n".encode())
            f.write(pgm_data)

        # 2. ساخت فایل YAML
        with open(yaml_file, 'w') as f:
            f.write(f"image: {pgm_file}\n")
            f.write(f"resolution: {self.resolution}\n")
            f.write(f"origin: [{self.origin_x}, {self.origin_y}, 0.0]\n")
            f.write("negate: 0\n")
            f.write("occupied_thresh: 0.65\n")
            f.write("free_thresh: 0.196\n")

        self.get_logger().info(f"Map successfully saved as {pgm_file} and {yaml_file} in your current terminal directory!")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CustomSLAM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()