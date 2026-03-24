import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("dynamo_minimal_sim")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_slam = get_package_share_directory("slam_toolbox")

    world_file = os.path.join(pkg, "worlds", "warehouse.sdf")
    xacro_file = os.path.join(pkg, "urdf", "ridgeback.urdf.xacro")
    slam_params = os.path.join(pkg, "config", "slam_params.yaml")
    rviz_config = os.path.join(pkg, "rviz", "mapping_view.rviz")

    env_models = AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(pkg, "meshes"))
    env_robot = AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(pkg))

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        xacro_file,
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": True}],
        output="screen",
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r {world_file}"}.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "ridgeback",
            "-topic", "robot_description",
            "-x", "-3.0",
            "-y", "0.0",
            "-z", "0.1",
            "-Y", "0.0",
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/scan_rear@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    odom_tf = Node(
        package="dynamo_minimal_sim",
        executable="odom_tf_pub",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_slam, "launch", "online_async_launch.py")),
        launch_arguments={
            "use_sim_time": "true",
            "slam_params_file": slam_params,
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        env_models,
        env_robot,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        odom_tf,
        slam,
        rviz,
    ])
