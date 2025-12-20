from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtle_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament 索引
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # ⭐ 安装 launch 文件（关键）
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yutou',
    maintainer_email='3186831775@qq.com',
    description='turtlesim navigation demo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_tf = turtle_nav.turtle_tf_node:main',
            'turtle_static_map = turtle_nav.turtle_static_map_node:main',
            'turtle_astar_planner = turtle_nav.turtle_astar_planner:main',
        ],
    },
)
