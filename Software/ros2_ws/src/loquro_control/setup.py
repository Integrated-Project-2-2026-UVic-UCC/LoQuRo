from glob import glob

from setuptools import find_packages, setup

package_name = 'loquro_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/routines', glob('routines/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nico',
    maintainer_email='nllortfer@gmail.com',
    description='Nodes to command the LoQuRo quadruped',
    license='GPL-3.0-or-later',
    entry_points={
        'console_scripts': [
            'robot_interface = loquro_control.robot_interface:main',
            'routine_player = loquro_control.routine_player:main',
            'policy_player = loquro_control.policy_player:main',
        ],
    },
)
