from setuptools import setup
import os
from glob import glob

package_name = "dynamo_minimal_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro") + glob("urdf/*.gazebo") + glob("urdf/*.urdf")),
        (os.path.join("share", package_name, "urdf", "accessories"), glob("urdf/accessories/*")),
        (os.path.join("share", package_name, "urdf", "configs"), glob("urdf/configs/*")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pratham",
    maintainer_email="pratham@example.com",
    description="Minimal Ridgeback Gazebo + LiDAR + SLAM occupancy grid project.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "odom_tf_pub = dynamo_minimal_sim.odom_tf_pub:main",
        ],
    },
)
