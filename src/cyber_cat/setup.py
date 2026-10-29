from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cyber_cat'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), 
            glob('worlds/*.world') + glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'urdf'), 
            glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'models'), 
            glob('models/*.sdf')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Sujie Zong',
    maintainer_email='zong.s@northeastern.edu',
    description='Vision-only air-ground pursuit: a TurtleBot 4 chases a Crazyflie',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_processor = cyber_cat.vision_processor_node:main',
            'chase_controller = cyber_cat.chase_controller_node:main',
            'mission_executive = cyber_cat.mission_executive_node:main',
            'voice_command = cyber_cat.voice_command_node:main',
            'crazyflie_patrol = cyber_cat.crazyflie_patrol_node:main',
        ],
    },
)
