from setuptools import setup
import os
from glob import glob

package_name = 'car_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (f'share/{package_name}/launch', glob('launch/*.py')),
        (f'share/{package_name}/urdf', glob('urdf/*.urdf')),
        (f'share/{package_name}', ['package.xml']),
    ],
)
