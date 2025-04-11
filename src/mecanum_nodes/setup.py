from setuptools import setup, find_packages

package_name = 'mecanum_nodes'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),  # <-- important fix
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'pyserial'],
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
            'stepper_pub = mecanum_nodes.stepper_pub:main',
        ],
    },
)



