from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로
    pkg_path = get_package_share_directory('tuto_package')
    urdf_file = os.path.join(pkg_path, 'urdf', 'tuto_robot.urdf')
    world_file = os.path.join(pkg_path, 'worlds', 'tuto.world')

    # URDF 내용 불러오기
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    return LaunchDescription([
        # Gazebo 실행
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # robot_state_publisher 실행
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description_content},
                {'use_sim_time': True}
            ],
            output='screen'
        ),

        # 로봇 스폰
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_urdf',
            arguments=[
                '-entity', 'my_robot',
                '-file', urdf_file,
                '-x', '0.0', '-y', '0.0', '-z', '0.3'
            ],
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
            output='screen'
        ),

        # 5. diff_drive_controller 실행
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_drive_controller'],
            output='screen'
        ),
    ])
