"""
Setup script for the rbsf_mvsim package.
"""
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rbsf_mvsim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch','*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, package_name),
            glob(os.path.join(package_name,'*world.xml'))),
        (os.path.join('share', package_name, package_name),
            glob(os.path.join(package_name,'*.png'))),
        (os.path.join('share', package_name, package_name),
            glob(os.path.join(package_name, '*.rviz'))),
        (os.path.join('share', package_name,'definitions'),
            glob(os.path.join('definitions', '*.xml'))),
        (os.path.join('share', package_name, 'models'),
            glob(os.path.join('models', '*'))),
        (os.path.join('share', package_name, 'rosbags'),
            glob(os.path.join('rosbags', 'traxxas*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'rbsf_mvsim'), glob('rbsf_mvsim/*.xml')),
        (os.path.join('share', package_name, 'rbsf_mvsim'), glob('rbsf_mvsim/*.yaml')),
        (os.path.join('share', package_name, 'rbsf_mvsim'), glob('rbsf_mvsim/*.png')),
    ],
    install_requires=['setuptools', 'ackermann_msgs'],
    zip_safe=True,
    maintainer='Ruben Lopez',
    maintainer_email='r.lopezr@edu.uah.es',
    description='ROS2 package for nvsim testing of planning and control as part of Ruben Lopez Final Degree Project',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'path_publisher = rbsf_mvsim.nodes.path_publisher:main',
            'waypoint_logger = rbsf_mvsim.nodes.waypoint_logger:main',
            'controller_node = rbsf_mvsim.nodes.controller_node:main',
            'metrics_evaluator = rbsf_mvsim.nodes.metrics_evaluator:main',
            'offline_planner_node = rbsf_mvsim.nodes.offline_planner_node:main',
            'auto_param_tuner = rbsf_mvsim.nodes.auto_param_tuner:main',
            'centerline_generator = rbsf_mvsim.nodes.centerline_generator:main',
            'ackermann_to_twist = rbsf_mvsim.nodes.ackermann_to_twist:main',
        ],
    },
)
