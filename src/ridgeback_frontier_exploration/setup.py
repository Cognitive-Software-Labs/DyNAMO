from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ridgeback_frontier_exploration'

def generate_data_files():
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ]
    # Recursively add 'urdf', 'meshes', 'worlds', 'materials', 'config', and 'rviz'
    for directory in ['urdf', 'meshes', 'worlds', 'materials', 'config', 'rviz']:
        if os.path.exists(directory):
            for (path, directories, filenames) in os.walk(directory):
                for filename in filenames:
                    file_path = os.path.join(path, filename)
                    install_path = os.path.join('share', package_name, path)
                    data_files.append((install_path, [file_path]))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=generate_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pratham',
    maintainer_email='i.m.pratham01@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_tf_pub = ridgeback_frontier_exploration.odom_tf_pub:main',
        ],
    },
)
