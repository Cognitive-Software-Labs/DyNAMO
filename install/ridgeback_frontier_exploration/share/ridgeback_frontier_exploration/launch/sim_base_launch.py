import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('ridgeback_frontier_exploration')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_slam = get_package_share_directory('slam_toolbox')

    xacro_file = os.path.join(pkg_share, 'urdf', 'ridgeback.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'hospital.sdf')
    rviz_file = os.path.join(pkg_share, 'rviz', 'nav2_view.rviz')
    slam_params = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    
    # Fix grafic standard pentru mediul Ubuntu
    set_qt = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')

    env_models = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg_share, 'models'))
    env_robot = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg_share))

    # 1. State Publisher (Inima robotului)
    robot_desc = Command([PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', xacro_file])
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': ParameterValue(robot_desc, value_type=str), 'use_sim_time': True}])

    # 2. Simulatorul Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file}'}.items())

    # 3. Bridge-ul de date
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                             '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                             '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                             '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
                  parameters=[{'use_sim_time': True}])

    # 4. Adăugarea fizică a robotului (Spawn) la T=4 secunde
    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-name', 'ridgeback', '-topic', 'robot_description', '-x', '0.0', '-y', '0.0', '-z', '0.2'])

    # 5. Fix TF și SLAM la T=7 secunde
    tf_fix = Node(package='ridgeback_frontier_exploration', executable='odom_tf_pub', parameters=[{'use_sim_time': True}])
    
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_slam, 'launch', 'online_async_launch.py')),
        launch_arguments={'use_sim_time': 'True', 'slam_params_file': slam_params}.items())

    # 6. RViz la T=10 secunde (Când restul s-a stabilizat)
    rviz = Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_file],
                parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        set_qt, env_models, env_robot, rsp, gz_sim, bridge,
        TimerAction(period=4.0, actions=[spawn]),
        TimerAction(period=7.0, actions=[tf_fix, slam]),
        TimerAction(period=10.0, actions=[rviz])
    ])