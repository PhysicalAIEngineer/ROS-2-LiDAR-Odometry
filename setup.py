from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'simple_lidar_odometry'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PhysicalAIEngineer',
    maintainer_email='chetansonigara01@gmail.com',
    description='ROS 2 LiDAR odometry using Open3D point-to-plane ICP for sequential point-cloud motion estimation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_odometry_node = simple_lidar_odometry.lidar_odometry_node:main',
        ],
    },
)
