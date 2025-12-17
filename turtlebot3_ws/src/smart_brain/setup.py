import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'smart_brain'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install package.xml and resource index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),

        # Install configuration files (CRITICAL STEP)
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),

        # Install map files (CRITICAL STEP for PGM/YAML lookup)
        (os.path.join('share', package_name, 'config'), glob('config/*.pgm') + glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='The smart_brain package for Turtlebot3 Nav2 and perception.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'send_goal = smart_brain.send_goal:main',
        	'integrated_navigator = smart_brain.integrated_navigator:main',
        ],
    },
)
