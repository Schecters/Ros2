from setuptools import setup
import os
from glob import glob

package_name = 'gocargo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # package info
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),

        # urdf + meshes
        ('share/' + package_name + '/urdf/AllPart_ROS_1/urdf', 
            ['urdf/AllPart_ROS_1/urdf/AllPart_ROS_1.urdf']),
        ('share/' + package_name + '/urdf/AllPart_ROS_1/meshes', 
            glob('urdf/AllPart_ROS_1/meshes/*')),

        # maps
        ('share/' + package_name + '/maps', glob('maps/*')),

        # rviz config
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nine',
    maintainer_email='your_email@example.com',
    description='Gocargo bringup package for TurtleBot3 with Nav2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [],
    },
)

