from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'risabot_v4_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@todo.todo',
    description='Guarded V4 trajectory execution and command handoff.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'motion_executor = risabot_v4_control.motion_executor:main',
        ],
    },
)
