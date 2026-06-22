from setuptools import setup
import os
from glob import glob

package_name = 'blaze_slam'

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
    maintainer='Israel Charles',
    maintainer_email='israelcharlesic10@gmail.com',
    description='SLAM Toolbox track mapping for blaze_racer with autosave on loop closure.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_autosaver = blaze_slam.map_autosaver:main',
        ],
    },
)
