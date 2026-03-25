import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    use_gz_gui = LaunchConfiguration("use_gz_gui")

    pkg = get_package_share_directory("dynamo_minimal_sim")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_slam = get_package_share_directory("slam_toolbox")

    world_file = os.path.join(pkg, "worlds", "hospital.sdf")
    xacro_file = os.path.join(pkg, "urdf", "ridgeback.urdf.xacro")
    slam_params = os.path.join(pkg, "config", "slam_params.yaml")
    rviz_config = os.path.join(pkg, "rviz", "mapping_view.rviz")

    # Environment variables for meshes
    pkg_share_path = get_package_share_directory("dynamo_minimal_sim")
    
    env_vars = [
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(pkg_share_path, "meshes")),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(pkg_share_path, "urdf")),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(pkg_share_path)),
    ]

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        xacro_file,
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            robot_description,
            {
                "use_sim_time": True,
            },
        ],
        output="screen",
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        condition=IfCondition(use_gz_gui),
        launch_arguments={"gz_args": f"-r {world_file}"}.items(),
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        condition=UnlessCondition(use_gz_gui),
        launch_arguments={"gz_args": f"-r -s {world_file}"}.items(),
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
            "/odom_raw@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/camera_d455/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
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
        condition=IfCondition(use_rviz),
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        SetEnvironmentVariable("LD_PRELOAD", ""),
        SetEnvironmentVariable("SNAP", ""),
        SetEnvironmentVariable("SNAP_NAME", ""),
        SetEnvironmentVariable("SNAP_INSTANCE_NAME", ""),
        SetEnvironmentVariable("SNAP_ARCH", ""),
        SetEnvironmentVariable("SNAP_LIBRARY_PATH", ""),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz2.",
        ),
        DeclareLaunchArgument(
            "use_gz_gui",
            default_value="true",
            description="Launch Gazebo GUI (set false for headless/server mode).",
        ),
        *env_vars,
        gz_sim_gui,
        gz_sim_headless,
        robot_state_publisher,
        spawn_robot,
        bridge,
        odom_tf,
        slam,
        rviz,
    ])


