from setuptools import setup
import os
from glob import glob

package_name = 'gesture_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bao',
    maintainer_email='baodz20811@gmail.com',
    description='Hand gesture recognition to control TurtleBot 4',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gesture_node = gesture_control.gesture_node:main',
            'behavior_node = gesture_control.behavior_node:main',
        ],
    },
)
