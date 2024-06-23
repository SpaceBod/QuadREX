import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('nav'), 'config')
    rviz_file = os.path.join(config_dir, 'rex_rviz.rviz')
   
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],  # Changed to list instead of dict
        output='screen'
    )
    
    return LaunchDescription([
        rviz_node
    ])
   
if __name__ == '__main__':
    generate_launch_description()

