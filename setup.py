import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'auto_landing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Autonomous Drone Landing System',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_detector = auto_landing.target_detector:main',
            'mission_commander = auto_landing.mission_commander:main',
            'flight_controller = auto_landing.flight_controller:main',
            'base_teleporter = auto_landing.base_teleporter:main',
        ],
    },
)
