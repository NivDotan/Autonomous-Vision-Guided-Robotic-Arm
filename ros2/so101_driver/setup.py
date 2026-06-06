from setuptools import setup
from glob import glob

package_name = 'so101_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='niv',
    maintainer_email='nivmika@gmail.com',
    description='SO-101 servo driver ROS2 node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'driver_node = so101_driver.driver_node:main',
        ],
    },
)
