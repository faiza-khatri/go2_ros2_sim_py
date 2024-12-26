import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, GroupAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetRemap
from launch.event_handlers import OnProcessExit
import xacro, yaml

def generate_launch_description():
    ld = LaunchDescription()


    package_name = 'go1_description'
    pkg_path = get_package_share_directory(package_name)
    robots_file_path = os.path.join(pkg_path, 'config', 'robots.yaml')

    # Создаем пустой список роботов
    robots = []

    # Загрузка данных из YAML файла
    with open(robots_file_path, 'r') as file:
        yaml_data = yaml.safe_load(file)

    # Добавление роботов из YAML файла в список
    robots.extend(yaml_data['robots'])



    

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Использовать симуляционное время'
    )

    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    declare_enable_rviz = DeclareLaunchArgument(
        name='enable_rviz', default_value=enable_rviz, description='Enable rviz launch'
    )


    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_use_sim_time)

    world_file = os.path.join(pkg_path, 'world', 'empty.world') 
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                launch_arguments={'gz_args': ['-r -v4 ', world_file], 'on_exit_shutdown': 'true'}.items()
        )
    ld.add_action(gazebo)

    remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            # ("/scan", "scan"),
            # ("/odom", "odom")
        ]    
    
    map_server = Node(package='nav2_map_server',
                      executable='map_server',
                      name='map_server',
                      output='screen',
                      parameters=[{'yaml_filename': os.path.join(get_package_share_directory('go1_description'), 'maps', 'warehouse_map.yaml'),
                                   }, ],
                      remappings=remappings)

    map_server_lifecycle = Node(package='nav2_lifecycle_manager',
                                executable='lifecycle_manager',
                                name='lifecycle_manager_map_server',
                                output='screen',
                                parameters=[{'use_sim_time': use_sim_time},
                                            {'autostart': True},
                                            {'node_names': ['map_server']}])

    # Добавляем map_server и map_server_lifecycle
    # ld.add_action(map_server)
    # ld.add_action(map_server_lifecycle)


    remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("/scan", "scan"),
            ("/odom", "odom")
        ]    
    
    last_action = None
    # Spawn turtlebot3 instances in gazebo
    for robot in robots:

        namespace = robot['name']
        robot_name = robot['name']
        xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')
        robot_desc = xacro.process_file(xacro_file, mappings={'robot_name': robot_name}).toxml()
        params_robot_state_publisher = {'robot_description': robot_desc, 'use_sim_time': use_sim_time}

        node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            namespace=namespace,
            parameters=[params_robot_state_publisher],
            remappings=remappings
        )






        spawn_entity = Node(package='ros_gz_sim', executable='create', namespace=namespace,
                            arguments=['-topic', f'/{namespace}/robot_description',
                                    '-name', f'/{namespace}/my_bot',
                                    '-x', robot['x_pose'],
                                    '-y', robot['y_pose'],
                                    '-z', robot['z_pose']],
                            output='screen')

        
        ros_gz_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=namespace,
            name='ros_gz_bridge',
            output='screen',
            arguments=[
                f'/{namespace}/imu_plugin/out@sensor_msgs/msg/Imu@gz.msgs.IMU',
                f'/{namespace}/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                # f'/{namespace}/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
                f'/{namespace}/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                f'/{namespace}/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'
            ]
        )


        joint_state_broadcaster = Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=['joint_state_broadcaster'],
            output='screen',
            remappings=remappings
        )

        joint_group_controller = Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=['joint_group_controller'],
            output='screen',
            remappings=remappings
        )

        controller = Node(
            package='go1_controller',
            executable='robot_controller_gazebo.py',
            name='quadruped_controller',
            namespace=namespace,
            output='screen',
            remappings=remappings
        )

        odom = Node(
            package='go1_controller',
            executable='QuadrupedOdometryNode.py',
            name='odom',
            namespace=namespace,
            output='screen',
            parameters=[{
                "verbose": False,
                'publish_rate': 50,
                'open_loop': False,
                'has_imu_heading': True,
                'is_gazebo': True,
                'imu_topic': f'/{namespace}/imu',
                'base_frame_id': "base",
                'odom_frame_id': "odom",
                'clock_topic': f'/clock',
                'enable_odom_tf': True,
            }],
            remappings=remappings
        )


        nav2_launch_file = os.path.join(pkg_path, 'launch', 'nav2', 'bringup_launch.py')
        map_yaml_file = os.path.join( get_package_share_directory(package_name), 'maps', 'warehouse_map.yaml')
        params_file = os.path.join(get_package_share_directory(package_name), 'config', 'nav2_params.yaml')
        
        message = f"{{header: {{frame_id: map}}, pose: {{pose: {{position: {{x: {robot['x_pose']}, y: {robot['y_pose']}, z: 0.1}}, orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}, }} }}"

        initial_pose_cmd = ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable', 
                namespace + '/initialpose',
                'geometry_msgs/PoseWithCovarianceStamped', message
            ],
            output='screen'
        )        
        
        bringup_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'map': map_yaml_file,
                'use_namespace': 'True',
                'namespace': namespace,
                'params_file': params_file,  
                'autostart': 'true',
                'use_sim_time': 'true',
                'log_level': 'warn',
                'map_server': 'True'
            }.items()
        )

        nav2_actions = GroupAction([
            SetRemap(src="/tf", dst="tf"),
            SetRemap(src="/tf_static", dst="tf_static"),
            bringup_cmd,
            initial_pose_cmd,
        ])

        robot_control = GroupAction([
            SetRemap(src="/tf", dst="tf"),
            SetRemap(src="/tf_static", dst="tf_static"),
            joint_state_broadcaster,
            joint_group_controller,
            controller,
            odom
        ])


        rviz_launch_file = os.path.join(pkg_path, 'launch', 'rviz_launch.py')
        rviz_config_file = os.path.join(pkg_path, 'config', 'multi_nav2_default_view.rviz')

        rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch_file),
            launch_arguments={
                "namespace": namespace,
                "use_namespace": 'true',
                "rviz_config": rviz_config_file,
            }.items(),
            condition=IfCondition(enable_rviz)
        )


        cmd_vel_pub = Node(
            package='go1_controller',
            executable='cmd_vel_pub.py',
            name='cmd_vel_pub',
            output='screen',
            remappings=remappings
        )


        if last_action is None:
            # Call add_action directly for the first robot to facilitate chain instantiation via RegisterEventHandler
            ld.add_action(node_robot_state_publisher)
            ld.add_action(spawn_entity)
            ld.add_action(ros_gz_bridge)
            ld.add_action(robot_control)
            # ld.add_action(nav2_actions)
            # ld.add_action(rviz)

        else:
            # Use RegisterEventHandler to ensure next robot creation happens only after the previous one is completed.
            # Simply calling ld.add_action for spawn_entity introduces issues due to parallel run.
            spawn_robot_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[
                        # rviz,
                        # nav2_actions,
                        robot_control,
                        ros_gz_bridge,
                        spawn_entity,
                        node_robot_state_publisher,
                    ],
                )
            )

            ld.add_action(spawn_robot_event)

        # Save last instance for next RegisterEventHandler
        last_action = spawn_entity

    return ld