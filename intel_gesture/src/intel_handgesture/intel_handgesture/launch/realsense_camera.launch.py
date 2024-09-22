from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    realsense_node = Node(
        package='realsense2_camera',         # Name of the RealSense ROS 2 package
        executable='realsense2_camera_node', # Name of the executable 
        name='my_realsense_camera',         # Give your camera node a descriptive name (optional) 
        output='screen',                     # Print node output to the terminal
        parameters=[
            {'json_file_path': '/home/yvonne/intel_gesture/src/intel_handgesture/intel_handgesture/realsense_config.json'} 
        ]
    )

    return LaunchDescription([realsense_node])
