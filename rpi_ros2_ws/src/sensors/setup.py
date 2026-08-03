import os

from setuptools import find_packages, setup

package_name = 'sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='感測邊界層：原始訊號 → 控制層可用的回饋格式',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'float32_to_float64_converter_node = sensors.float32_to_float64_converter_node:main',
            'imu_to_orientation_node = sensors.imu_to_orientation_node:main',
        ],
    },
)
