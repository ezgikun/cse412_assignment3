from setuptools import find_packages, setup

package_name = 'command_server_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'custom_interfaces'],
    zip_safe=True,
    maintainer='ezgialtiok',
    maintainer_email='ezgi.altiok@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
'command_server = command_server_pkg.command_server:main',
        ],
    },
)
