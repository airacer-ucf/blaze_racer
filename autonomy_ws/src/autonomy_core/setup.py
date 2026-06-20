from setuptools import setup

package_name = 'autonomy_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Israel Charles',
    maintainer_email='israelcharlesic10@gmail.com',
    description='Shared base classes and LiDAR utilities for autonomy algorithms',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
