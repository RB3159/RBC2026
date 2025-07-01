from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_sim')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': os.path.join(pkg_share, 'worlds', 'simple.world'),
            'gui': 'true',
            'paused': 'false',
            'use_sim_time': 'true',
        }.items(),
    )

    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
        {'robot_description': robot_description},
        {'use_sim_time': True}
        ],
        output='screen'
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'simple_robot',
                   '-file', urdf_file,
                   '-x', '0', '-y', '0', '-z', '0.3'],
        output='screen'
    )
    
    # 컨트롤러 스포너가 필요 없으면 주석처리
    # controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_controller', 'steering_controller', 'driving_controller', 'lifting_controller'],
    #     output='screen'
    # )

    ld = LaunchDescription()
    ld.add_action(gazebo_launch)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_node)
    # ld.add_action(controller_spawner)  # 컨트롤러 필요 시 활성화

    return ld

