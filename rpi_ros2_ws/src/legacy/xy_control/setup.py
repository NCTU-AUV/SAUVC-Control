import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'xy_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bottom_camera_pid_bridge_node = xy_control.bottom_camera_pid_bridge_node:main',
            'lk_total_transform_node = xy_control.lk_total_transform_node:main',
            'waypoint_target_publisher = xy_control.waypoint_target_publisher:main',
            'yaw_reference_unwrapper_node = xy_control.yaw_reference_unwrapper_node:main',
            'dive_then_forward_mission_node = xy_control.dive_then_forward_mission_node:main',
        ],
    },
)
