from setuptools import setup
from glob import glob

package_name = 'so101_perception'

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
    description='SO-101 camera, sensor, and AI perception nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'base_camera_node    = so101_perception.base_camera_node:main',
            'gripper_camera_node = so101_perception.gripper_camera_node:main',
            'distance_sensor_node = so101_perception.distance_sensor_node:main',
            'object_detection_node = so101_perception.object_detection_node:main',
            'sam_node            = so101_perception.sam_node:main',
            'drop_zone_detector_node = so101_perception.drop_zone_detector_node:main',
        ],
    },
)
