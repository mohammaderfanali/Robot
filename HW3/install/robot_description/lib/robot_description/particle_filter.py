#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, TransformStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import TransformBroadcaster
import tf_transformations
import numpy as np
import math

class ParticleFilter(Node):
    def __init__(self):
        super().__init__('particle_filter')

        # پارامترهای ذرات
        self.num_particles = 200     # تعداد ذرات
        self.particles = []          # لیست ذرات [x, y, theta, weight]
        
        # وضعیت اولیه
        self.map_data = None
        self.resolution = 0.05
        self.width = 0
        self.height = 0
        self.origin = [0, 0]
        self.last_odom = None
        self.initialized = False

        # نویزهای حرکت (قابل تنظیم)
        self.alpha_rot = 0.1   # نویز چرخش
        self.alpha_trans = 0.1 # نویز حرکت مستقیم

        # ایجاد Publisherها
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.cloud_pub = self.create_publisher(PoseArray, '/particlecloud', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        

        # ایجاد Subscriberها
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Odometry, '/ekf_diff_imu/odom', self.odom_callback, odom_qos)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.get_logger().info("Particle Filter Node Started via ROS 2 Jazzy...")

    def map_callback(self, msg):
        # دریافت نقشه (فقط یکبار یا هنگام تغییر)
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        self.origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        
        if not self.initialized:
            self.initialize_particles()

    def initialize_particles(self):
        # پخش کردن ذرات به صورت تصادفی در فضای آزاد نقشه
        self.particles = []
        attempts = 0
        while len(self.particles) < self.num_particles and attempts < 10000:
            rx = np.random.uniform(0, self.width)
            ry = np.random.uniform(0, self.height)
            
            # چک کردن اینکه خانه نقشه خالی باشد (0)
            if self.map_data[int(ry), int(rx)] == 0:
                wx = rx * self.resolution + self.origin[0]
                wy = ry * self.resolution + self.origin[1]
                theta = np.random.uniform(-math.pi, math.pi)
                self.particles.append([wx, wy, theta, 1.0])
            attempts += 1
            
        self.initialized = True
        self.get_logger().info(f"Initialized {len(self.particles)} particles.")
        self.publish_particles()

    def odom_callback(self, msg):
        # مدل حرکت (Motion Model)
        if not self.initialized: return

        # گرفتن موقعیت فعلی از اودومتری
        q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        current_odom = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])

        if self.last_odom is not None:
            # محاسبه میزان جابجایی ربات (Delta)
            dx = current_odom[0] - self.last_odom[0]
            dy = current_odom[1] - self.last_odom[1]
            dtheta = current_odom[2] - self.last_odom[2]

            # اگر حرکتی انجام شده باشد
            dist = math.sqrt(dx**2 + dy**2)
            if dist > 0.001 or abs(dtheta) > 0.001:
                # حرکت دادن همه ذرات طبق حرکت ربات + نویز
                # چرخش بردار حرکت به مختصات لوکال ربات
                cos_a = math.cos(self.last_odom[2])
                sin_a = math.sin(self.last_odom[2])
                d_trans = dx * cos_a + dy * sin_a
                
                new_particles = []
                for p in self.particles:
                    # اضافه کردن نویز گوسی
                    noise_t = d_trans + np.random.normal(0, self.alpha_trans * dist)
                    noise_r = dtheta + np.random.normal(0, self.alpha_rot * abs(dtheta))
                    
                    p[2] += noise_r
                    p[0] += noise_t * math.cos(p[2])
                    p[1] += noise_t * math.sin(p[2])
                    
                    # نرمال کردن زاویه
                    p[2] = math.atan2(math.sin(p[2]), math.cos(p[2]))
                    new_particles.append(p)
                
                self.particles = new_particles
                self.publish_particles()

        self.last_odom = current_odom

    def scan_callback(self, msg):
        # مدل سنسور (Sensor Model) و ریسمپلینگ
        if not self.initialized or self.map_data is None: return

        # ساده سازی داده‌های اسکن برای سرعت (هر 10 پرتو یکی)
        self.get_logger().info(f'Laser Scan received! Number of beams: {len(msg.ranges)}', throttle_duration_sec=2.0)
        step = 10
        ranges = msg.ranges[::step]
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment * step)

        weights = []
        total_w = 0.0

        for p in self.particles:
            score = 0.0
            valid_points = 0
            
            # بررسی تطابق لیزر مجازی ذره با نقشه
            for r, ang in zip(ranges, angles):
                if r < msg.range_min or r > msg.range_max: continue
                
                # نقطه برخورد لیزر در مختصات جهانی
                hit_x = p[0] + r * math.cos(p[2] + ang)
                hit_y = p[1] + r * math.sin(p[2] + ang)
                
                # تبدیل به مختصات نقشه
                mx = int((hit_x - self.origin[0]) / self.resolution)
                my = int((hit_y - self.origin[1]) / self.resolution)
                
                if 0 <= mx < self.width and 0 <= my < self.height:
                    # اگر خانه نقشه اشغال است (دیوار)، امتیاز مثبت
                    if self.map_data[my, mx] > 50:
                        score += 1.0
                    valid_points += 1
            
            # محاسبه وزن نهایی
            w = (score / (valid_points + 1)) ** 2  # توان 2 برای حساسیت بیشتر
            weights.append(w + 1e-10) # جلوگیری از صفر مطلق
            
        # نرمال سازی وزن ها
        # نرمال سازی وزن ها
       # نرمال سازی وزن ها
        total_w = sum(weights)
        weights = [w/total_w for w in weights]
        
        # محاسبه شاخص تنوع ذرات (N_eff)
        n_eff = 1.0 / sum([w**2 for w in weights])
        
      # فقط اگر تنوع ذرات خیلی کم شد ریسمپل کن
        if n_eff < self.num_particles / 2.0:
            
            # --- ایده تزریق ذرات جدید ---
            keep_ratio = 0.90  # 90 درصد ذرات را از قبل نگه می‌داریم
            num_keep = int(self.num_particles * keep_ratio)
            num_random = self.num_particles - num_keep  # 10 درصد ذرات کاملاً جدید می‌سازیم

            # 1. انتخاب ذرات خوب (90 درصد) بر اساس وزن‌ها
            indices = np.random.choice(len(self.particles), num_keep, p=weights)
            
            new_particles = []
            for i in indices:
                p = list(self.particles[i]) 
                # کمی نویز برای جلوگیری از روی هم افتادن دقیق ذرات تکراری
                p[0] += np.random.normal(0, 0.05) 
                p[1] += np.random.normal(0, 0.05) 
                p[2] += np.random.normal(0, 0.05) 
                new_particles.append(p)
            
            # 2. تزریق ذرات کاملاً رندوم در فضای آزاد نقشه (10 درصد)
            attempts = 0
            injected = 0
            while injected < num_random and attempts < 2000:
                rx = np.random.uniform(0, self.width)
                ry = np.random.uniform(0, self.height)
                
                # بررسی اینکه نقطه تصادفی روی دیوار نباشد
                if self.map_data[int(ry), int(rx)] == 0: 
                    wx = rx * self.resolution + self.origin[0]
                    wy = ry * self.resolution + self.origin[1]
                    theta = np.random.uniform(-math.pi, math.pi)
                    # اضافه کردن ذره جدید با وزن پایه
                    new_particles.append([wx, wy, theta, 1.0 / self.num_particles])
                    injected += 1
                attempts += 1
            
            # جایگزینی کل ذرات با لیست جدید
            self.particles = new_particles
        
        # محاسبه مکان نهایی و انتشار آن
        self.estimate_and_publish_pose()

    def estimate_and_publish_pose(self):
        # محاسبه میانگین ذرات (مکان تخمینی ربات)
        x_avg = np.mean([p[0] for p in self.particles])
        y_avg = np.mean([p[1] for p in self.particles])
        
        # میانگین گیری زاویه (روش برداری)
        sin_avg = np.mean([math.sin(p[2]) for p in self.particles])
        cos_avg = np.mean([math.cos(p[2]) for p in self.particles])
        yaw_avg = math.atan2(sin_avg, cos_avg)
        
        # انتشار Pose
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x_avg
        msg.pose.pose.position.y = y_avg
        q = tf_transformations.quaternion_from_euler(0, 0, yaw_avg)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        self.pose_pub.publish(msg)
        
   
    def publish_particles(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        for p in self.particles:
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            q = tf_transformations.quaternion_from_euler(0, 0, p[2])
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            msg.poses.append(pose)
        self.cloud_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ParticleFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()