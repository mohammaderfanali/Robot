import os
from glob import glob
from setuptools import find_packages, setup
package_name = 'robot_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mohammadreza',
    maintainer_email='mohamamdgwentgame@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ['imu_bias_corrector = robot_estimation.imu_bias_corrector:main', 
            'lowpass_filter = robot_estimation.lowpass_filter:main',  
            'complementary_filter = robot_estimation.complementary_filter:main', 
            'diff_drive_controller = robot_estimation.diff_drive_controller:main',
            'odom_producer = robot_estimation.odom_producer:main',
        ],
    },
)
