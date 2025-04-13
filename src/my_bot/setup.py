from setuptools import setup
import os
from glob import glob

package_name = 'my_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'description'), glob('description/*.xacro')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml')),
    ],
    install_requires=['setuptools', 'tf_transformations'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Minimal planner goal sender node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_publisher_node = my_bot.goal_publisher_node:main',
            'filtered_path_publisher_node = my_bot.filtered_path_publisher_node:main',
        ],
    },
)
