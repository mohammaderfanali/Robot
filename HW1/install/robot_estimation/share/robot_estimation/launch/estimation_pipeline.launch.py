from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for the IMU processing pipeline.
    
    Data Flow:
    /zed/zed_node/imu/data_raw -> imu_bias_corrector -> /imu/data_corrected
    /imu/data_corrected -> lowpass_filter -> /imu/data_filtered
    /imu/data_filtered -> complementary_filter -> /estimation/orientation
    """
    
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
        bias_corrector_node,
        lowpass_filter_node,
        complementary_filter_node,
    ])