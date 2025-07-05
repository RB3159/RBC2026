from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('car_test')
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_car.urdf')
    with open(urdf_file, 'r') as f:
        urdf_str = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_str}]
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'my_car',
                '-file', urdf_file
            ],
            output='screen'
        ),
    ])
