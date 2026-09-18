from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'risabot_v4_experimental'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, [
            'package.xml', 'README.md', 'PORTING_PLAN.md', 'CALIBRATION.md',
            'PARKING_GOAL.md', 'THIRD_PARTY_NOTICES.md'
        ]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@todo.todo',
    description='Isolated shadow-mode port of the Carbot Simulator V4 algorithms.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'shadow_monitor = risabot_v4_experimental.shadow_monitor:main',
            'bev_shadow = risabot_v4_experimental.bev_shadow:main',
            'calibrate_intrinsics = risabot_v4_experimental.calibrate_intrinsics:main',
            'road_mask_shadow = risabot_v4_experimental.road_mask_shadow:main',
            'pose_shadow = risabot_v4_experimental.pose_shadow:main',
            'trajectory_shadow = risabot_v4_experimental.trajectory_shadow:main',
            'parking_shadow = risabot_v4_experimental.parking_shadow:main',
        ],
    },
)
