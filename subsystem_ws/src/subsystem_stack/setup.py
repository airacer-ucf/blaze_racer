from setuptools import setup
import os
from glob import glob

package_name = 'subsystem_stack'

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
    description='Onboard drivers for vesc and sensors for small-scale vehicles. Based on RoboRacer (F1Tenth) stack.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'throttle_interpolator = subsystem_stack.throttle_interpolator:main',
            'tf_publisher = subsystem_stack.tf_publisher:main'
        ],
    },
)
