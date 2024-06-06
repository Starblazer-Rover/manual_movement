import os
from glob import glob
from setuptools import setup

package_name = 'manual_movement'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='billee',
    maintainer_email='lbsaikali@cpp.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = manual_movement.controller:main',
            'joystick = manual_movement.joystick:main',
            'rover_movement = manual_movement.neo_teensy:main',
            'autonomous = manual_movement.autonomous_movement:main',
            'encoder = manual_movement.encoders:main',
            'arm = manual_movement.arm_subscriber:main',
        ],
    },
)
