import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tuto = get_package_share_directory('tuto_package')
    rviz_config_file = os.path.join(pkg_tuto, 'rviz', 'nav2_config.rviz')

    # Gazebo 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': os.path.join(pkg_tuto, 'worlds', 'home.world')}.items()
    )

    # URDF 로드
    urdf_file = os.path.join(pkg_tuto, 'urdf', 'tuto_half_home.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description_content}]
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'tuto_robot', '-x', '7', '-y', '1.5', '-z', '0.3'],
        output='screen'
    )

    odom_tf_broadcaster_node = Node(
        package='tuto_package',
        executable='odom_tf_broadcaster',
        name='odom_tf_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    teleop_twist_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e'
    )

    # Map Server 명시적 실행
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': os.path.join(pkg_tuto, 'maps', 'my_home_map.yaml'),
            'frame_id': 'map',
            'topic_name': '/map',
            'always_send_full_map': True,
            'subscribe_transient_local': True  # 🔥 이거!
        }]
    )

    detector_node = Node(
        package='tuto_package',  # 실제 패키지명으로 교체
        executable='detector_node',
        name='detector_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Lifecycle Manager - map_server 전용
    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen'
    )

    # AMCL 노드
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[os.path.join(pkg_tuto, 'config', 'nav2_params.yaml')],
        remappings=[('/scan', '/lidar/scan')]
    )

    # Lifecycle Manager - AMCL 전용
    lifecycle_manager_amcl = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )
    
    nav2_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': os.path.join(pkg_tuto, 'maps', 'my_home_map.yaml'),
            'use_sim_time': 'True',
            'autostart': 'True',
            'params_file': os.path.join(pkg_tuto, 'config', 'nav2_params.yaml')
        }.items()
    )

    # Nav2 bringup (map_server & amcl은 별도 실행되므로)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': os.path.join(pkg_tuto, 'config', 'nav2_params.yaml'),
            'autostart': 'True',
            'map_subscribe_transient_local': 'true'
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            os.path.join(pkg_tuto, 'config', 'nav2_params.yaml'),
            {'use_sim_time': True,
            'default_bt_xml_filename': os.path.join(pkg_tuto, 'my_bt_tree', 'my_bt_tree.xml')}
        ]
    )
    
    mpc_controller = Node(
            package='tuto_package',
            executable='mpc_node',
            name='mpc_controller',
            output='screen',
        )
    
    autonomy_node = Node(
        package='tuto_package',
        executable='autonomy_node',
        name='autonomy_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'bt_xml_file': 'my_bt_tree.xml'
        }]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
        odom_tf_broadcaster_node,
        teleop_twist_keyboard_node,
        nav2_localization_launch,
        nav2_launch,
        rviz_node,
        map_server_node
    ])
