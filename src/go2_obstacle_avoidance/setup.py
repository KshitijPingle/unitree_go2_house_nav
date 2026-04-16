from setuptools import setup
from glob import glob
import os

package_name = 'go2_obstacle_avoidance'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Martin Nguyen',
    maintainer_email='martinnguyen343@gmail.com',
    description='Obstacle avoidance safety layer for Unitree GO2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoidance_node = go2_obstacle_avoidance.obstacle_avoidance_node:main',
        ],
    },
)
