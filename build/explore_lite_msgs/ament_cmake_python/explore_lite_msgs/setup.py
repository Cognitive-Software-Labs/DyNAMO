from setuptools import find_packages
from setuptools import setup

setup(
    name='explore_lite_msgs',
    version='1.0.0',
    packages=find_packages(
        include=('explore_lite_msgs', 'explore_lite_msgs.*')),
)
