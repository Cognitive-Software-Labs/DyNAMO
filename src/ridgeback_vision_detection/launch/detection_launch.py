import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_vision_detection = get_package_share_directory('ridgeback_vision_detection')

    # Path to the red sphere SDF
    sdf_model_path = os.path.join(pkg_vision_detection, 'models', 'red_sphere.sdf')
    world_file = os.path.join(pkg_vision_detection, 'worlds', 'warehouse.sdf')

    # Launch world
    sim_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # Spawn Red Sphere (using ros_gz_sim create)
    spawn_sphere = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'red_sphere',
            '-file', sdf_model_path,
            '-x', '1.0',
            '-y', '-0.5',
            '-z', '0.2'
        ],
        output='screen',
    )

    # Vision Detection Node
    detector_node = Node(
        package='ridgeback_vision_detection',
        executable='detector',
        name='vision_detector',
        output='screen'
    )

    return LaunchDescription([
        sim_world,
        spawn_sphere,
        detector_node
    ])
