import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'manual_ctrl'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seeed',
    maintainer_email='seeed@todo.todo',
    description='Gamepad manual control with a latching exclusive takeover.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manual_control_node = manual_ctrl.manual_control_node:main',
            'manual_mux_node = manual_ctrl.manual_mux_node:main',
        ],
    },
)
