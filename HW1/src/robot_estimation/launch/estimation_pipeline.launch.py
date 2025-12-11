from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for the integrated pipeline (Control + IMU Processing).
    
    Data Flow:
    1. Control: /cmd_vel -> diff_drive_controller -> RPM -> odom_producer -> /wheel_encoder/odom_raw
    2. IMU: /zed/zed_node/imu/data_raw -> imu_bias_corrector -> lowpass_filter -> complementary_filter
    """
    
    
    diff_drive_controller_node = Node(
        package='robot_estimation',
        executable='diff_drive_controller', 
        name='diff_drive_controller',
        output='screen',
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('rpm_wheel_right', 'robot/rpm_right'),
            ('rpm_wheel_left', 'robot/rpm_left'),
        ]
    )

    odom_producer_node = Node(
        package='robot_estimation',
        executable='odom_producer', 
        name='odom_producer',
        output='screen',
        remappings=[
            ('robot/rpm_right', 'robot/rpm_right'),
            ('robot/rpm_left', 'robot/rpm_left'),
            ('wheel_encoder/odom_raw', 'wheel_encoder/odom_raw'), 
        ]
    )


    bias_corrector_node = Node(
        package='robot_estimation',
        executable='imu_bias_corrector', 
        name='imu_bias_corrector',
        output='screen',
        parameters=[{
            'sampling_duration_sec': 5.0,  
            'sample_rate_hz': 100.0,       
        }],
       
        remappings=[
            ('/zed/zed_node/imu/data_raw', '/zed/zed_node/imu/data_raw'),
          
        ]
    )

    lowpass_filter_node = Node(
        package='robot_estimation',
        executable='lowpass_filter',
        name='lowpass_filter',
        output='screen',
        parameters=[{
            'lowpass_alpha': 0.1, 
        }],
       
    )

    complementary_filter_node = Node(
        package='robot_estimation',
        executable='complementary_filter',
        name='complementary_filter',
        output='screen',
        parameters=[{
            'tau': 0.02, 
        }],
        remappings=[
            ('/estimation/orientation', 'estimation/orientation'),
        ]
    )
    
    return LaunchDescription([
        diff_drive_controller_node,
        odom_producer_node,
        bias_corrector_node,
        lowpass_filter_node,
        complementary_filter_node,
    ])