import os
from glob import glob

from setuptools import setup

package_name = 'blaze_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Israel Charles',
    maintainer_email='israelcharlesic10@gmail.com',
    description='slam_toolbox track mapping for blaze_racer (Lab 4).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_tf_publisher = blaze_slam.odom_tf_publisher:main',
        ],
    },
)
