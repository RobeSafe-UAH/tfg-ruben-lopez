from setuptools import find_packages, setup

package_name = 'rbsf_mvsim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fernando Villa',
    maintainer_email='fernando.villa@edu.uah.es',
    description='ROS2 package for saving some custom mvsim worlds developed as part of Fernando Villa`s final degree project',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vel_set = rbsf_mvsim.vel_set:main',
            'lidar_logger = rbsf_mvsim.lidar_logger_node:main',
        ],
    },
)
