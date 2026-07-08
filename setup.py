from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mf_action_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BetterOIer',
    maintainer_email='betteroier@github.com',
    description='ROS2 path and action planner for Merlin area with DFS algorithm and web-based UI',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dfs_planner_node = app.dfs_planner_node:main',
            'mf_buffer_node = app.mf_buffer_node:main',
            'monitor_node = app.monitor_node:main',
        ],
    },
)
