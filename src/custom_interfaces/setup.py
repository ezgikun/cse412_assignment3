import os
from glob import glob 

from setuptools import setup

package_name = 'custom_interfaces'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name], 
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ezgialtiok',
    maintainer_email='ezgi.altiok@hotmail.com',
    description='Custom messages and services for CSE412 Assignment 3',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
