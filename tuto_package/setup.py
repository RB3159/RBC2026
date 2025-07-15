from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tuto_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a',
    maintainer_email='a@a.com',
    description='Package for the tuto robot simulation.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_tf_broadcaster = tuto_package.odom_tf_broadcaster:main',
            'lidar_projection_node = tuto_package.lidar_projection_node:main',
            'detector_node = tuto_package.detector_node:main',
        ],
    },
)