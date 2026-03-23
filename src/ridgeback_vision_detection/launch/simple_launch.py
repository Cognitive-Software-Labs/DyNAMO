import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, AppendEnvironmentVariable,
                            TimerAction, ExecuteProcess)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_robot  = get_package_share_directory('ridgeback_vision_detection')
    pkg_slam   = get_package_share_directory('slam_toolbox')

    xacro_file  = os.path.join(pkg_robot, 'urdf', 'ridgeback.urdf.xacro')
    world_file  = os.path.join(pkg_robot, 'worlds', 'hospital.sdf')
    nav2_params = os.path.join(pkg_robot, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(pkg_robot, 'config', 'slam_params.yaml')
    rviz_config = os.path.join(pkg_robot, 'rviz', 'nav2_view.rviz')

    env_robot = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg_robot))

    robot_desc = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', xacro_file])
    robot_description = {
        'robot_description': ParameterValue(robot_desc, value_type=str)}

    # ── Robot State Publisher (T=0) ──
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}])

    # ── Gazebo on Xvfb (T=0) ──
    # Gazebo GUI crashes on this system (snap libpthread conflict).
    # xvfb-run gives ogre2 a virtual GL context so camera sensors render
    # properly (not all-white).  No visible Gazebo window — use RViz instead.
    gz_sim = ExecuteProcess(
        cmd=['xvfb-run', '-a', 'gz', 'sim', '-r', world_file],
        output='screen')

    # ── Static TF bridges (T=0) ──
    lidar_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0',
                   'lidar_front_link','ridgeback/base_link/lidar'],
        parameters=[{'use_sim_time': True}], output='log')
    camera_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0',
                   'd455_link','ridgeback/base_link/camera_d455'],
        parameters=[{'use_sim_time': True}], output='log')

    # ── Spawn robot (T=10) ──
    spawn = TimerAction(period=10.0, actions=[
        Node(package='ros_gz_sim', executable='create',
             arguments=['-name','ridgeback','-topic','robot_description',
                        '-x','-3.0','-y','0.0','-z','0.2'],
             output='screen')])

    # ── Bridge (T=15) ──
    bridge = TimerAction(period=15.0, actions=[
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=[
                 '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                 '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                 '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                 '/scan_rear@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                 '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                 '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                 '/camera_d455/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                 '/camera_d455/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
             ],
             parameters=[{'use_sim_time': True}], output='screen',
             remappings=[
                 ('/camera_d455/image_raw', '/camera/image_raw'),
                 ('/camera_d455/camera_info', '/camera/camera_info')])])

    # ── Odom TF (T=17) ──
    odom_tf = TimerAction(period=17.0, actions=[
        Node(package='ridgeback_vision_detection', executable='odom_tf_pub',
             name='odom_tf_publisher', output='screen',
             parameters=[{'use_sim_time': True}])])

    # ── SLAM (T=20) ──
    slam = TimerAction(period=20.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_slam, 'launch', 'online_async_launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'slam_params_file': slam_params}.items())])

    # ── Nav2 (T=25) ──
    nav2 = TimerAction(period=25.0, actions=[
        Node(package='nav2_controller', executable='controller_server',
             output='screen', parameters=[nav2_params],
             remappings=[('/tf','tf'),('/tf_static','tf_static')]),
        Node(package='nav2_smoother', executable='smoother_server',
             output='screen', parameters=[nav2_params],
             remappings=[('/tf','tf'),('/tf_static','tf_static')]),
        Node(package='nav2_planner', executable='planner_server',
             output='screen', parameters=[nav2_params],
             remappings=[('/tf','tf'),('/tf_static','tf_static')]),
        Node(package='nav2_behaviors', executable='behavior_server',
             output='screen', parameters=[nav2_params],
             remappings=[('/tf','tf'),('/tf_static','tf_static')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             output='screen', parameters=[nav2_params],
             remappings=[('/tf','tf'),('/tf_static','tf_static')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             output='screen', parameters=[{
                 'use_sim_time': True, 'autostart': True,
                 'node_names': ['controller_server','smoother_server',
                                'planner_server','behavior_server',
                                'bt_navigator']}])])

    # ── RViz2 (T=22) ──
    rviz = TimerAction(period=22.0, actions=[
        Node(package='rviz2', executable='rviz2', name='rviz2',
             output='screen', arguments=['-d', rviz_config],
             parameters=[{'use_sim_time': True}])])

    # ── Mission detector (T=40) ──
    detector = TimerAction(period=40.0, actions=[
        Node(package='ridgeback_vision_detection', executable='detector',
             name='hospital_mission', output='screen',
             parameters=[{'use_sim_time': True}])])

    # ── rqt image view (T=43) ──
    rqt = TimerAction(period=43.0, actions=[
        ExecuteProcess(
            cmd=['ros2','run','rqt_image_view','rqt_image_view',
                 '/vision_detection/image_result'],
            output='screen')])

    return LaunchDescription([
        env_robot, gz_sim, rsp, lidar_tf, camera_tf,
        spawn, bridge, odom_tf, slam, rviz, nav2,
        detector, rqt,
    ])
