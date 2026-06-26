# Copyright 2026 Diego Eduardo Martinez Cruz
# SPDX-License-Identifier: MIT
from glob import glob

from setuptools import find_packages, setup

package_name = 'quantum_rover_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Diego Eduardo Martinez Cruz',
    maintainer_email='diego.martinez111213@gmail.com',
    description='Hybrid quantum-classical actuator control nodes for an autonomous rover.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_simulator = quantum_rover_control.nodes.rover_simulator_node:main',
            'quantum_controller = quantum_rover_control.nodes.quantum_controller_node:main',
            'telemetry_logger = quantum_rover_control.nodes.telemetry_logger_node:main',
            'sim_demo = quantum_rover_control.sim_demo:main',
        ],
    },
)
