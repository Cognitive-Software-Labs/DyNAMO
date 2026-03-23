import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_robot = get_package_share_directory('ridgeback_vision_detection')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_slam = get_package_share_directory('slam_toolbox')

    xacro_file = os.path.join(pkg_robot, 'urdf', 'ridgeback.urdf.xacro')
    world_file = os.path.join(pkg_robot, 'worlds', 'warehouse.sdf')

    # ===============================
    # Gazebo resource path
    # ===============================
    env_models = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_robot, 'models')
    )

    env_robot = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.dirname(pkg_robot)
    )

    # ===============================
    # Robot description
    # ===============================
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        xacro_file
    ])

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen'
    )

    # ===============================
    # Gazebo
    # ===============================
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'ridgeback',
            '-topic', 'robot_description',
            '-x', '-15.0',
            '-y', '0.0',
            '-z', '0.1',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # ===============================
    # ROS ↔ Gazebo Bridge
    # ===============================
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # cmd_vel
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',

            # odom
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',

            # lidar
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/scan_rear@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',

            # clock
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',

            # camera
            '/camera_d455/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ===============================
    # SLAM
    # ===============================
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # ===============================
    # Nav2
    # ===============================
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true'
        }.items()
    )

    # ===============================
    # Vision Node
    # ===============================
    detector_node = Node(
        package='ridgeback_vision_detection',
        executable='detector',
        name='vision_detector',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        env_models,
        env_robot,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        slam,
        nav2,
        detector_node
    ])
