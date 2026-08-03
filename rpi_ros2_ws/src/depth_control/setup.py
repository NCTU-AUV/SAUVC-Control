import os

from setuptools import find_packages, setup

package_name = 'depth_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='深度軸：把 PID 的下沉力轉成 wrench 匯流排上的一路來源',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "output_sink_force_to_output_wrench_node = depth_control.output_sink_force_to_output_wrench_node:main",
        ],
    },
)
