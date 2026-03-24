import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_robot = get_package_share_directory('ridgeback_frontier_exploration')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_slam = get_package_share_directory('slam_toolbox')
    explore_params = os.path.join(pkg_robot, 'config', 'explore.yaml')

    xacro_file = os.path.join(pkg_robot, 'urdf', 'ridgeback.urdf.xacro')
    world_file = os.path.join(pkg_robot, 'worlds', 'hospital.sdf')

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
            '-x', '0.0',
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

            # clock
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',

            # camera (IMPORTANT: Use actual Gazebo topic)
            '/world/hospital/model/ridgeback/link/camera_link/sensor/camera/image'
            '@sensor_msgs/msg/Image@gz.msgs.Image'
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
    # Explore Lite Node
    # ===============================
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        parameters=[explore_params, {'use_sim_time': True}],
        output='screen'
    )

    # ===============================
    # RViz2
    # ===============================
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        # ==========================================
        # WAVE 1: T=0 Seconds (The Foundation)
        # Start Gazebo, load the URDF, and bridge the topics.
        # ==========================================
        env_models,
        env_robot,
        robot_state_publisher,
        gz_sim,
        bridge,

        # ==========================================
        # WAVE 2: T=5 Seconds (The Physical Robot)
        # Give Gazebo 5 seconds to wake up, THEN spawn the robot.
        # ==========================================
        TimerAction(
            period=5.0, 
            actions=[spawn_robot]
        ),

        # ==========================================
        # WAVE 3: T=10 Seconds (The Brains)
        # Give the robot 5 seconds to fall to the ground and start 
        # spinning its LiDAR, THEN start SLAM, Nav2, and Explore.
        # ==========================================
        TimerAction(
            period=10.0, 
            actions=[slam, nav2, explore_node]
        )
    ])