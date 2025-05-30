from setuptools import setup
from glob import glob

package_name = 'mecanum_nodes'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@example.com',
    description='Mecanum kinematics planner and controller ROS2 nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'planner = mecanum_nodes.planner_node:main',
            'controller = mecanum_nodes.controller_node:main',
            'serial_sub = mecanum_nodes.serial_subscriber:main',
        ],
    },
)


