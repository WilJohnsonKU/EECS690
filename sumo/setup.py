from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sumo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch/'),
        glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='11306260+liangfuyuan@user.noreply.gitee.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'attack_enemy = sumo.attack_node:main',
            'square_controller = sumo.square_controller:main',
            'wall_avoider = sumo.lidar_wall_avoider:main',
            'finder_node = sumo.finder_node:main',
            'control = sumo.control_node:main',
            'centroid = sumo.centroid_finder:main',
            'safety = sumo.safety_node:main',
            'imu = sumo.imu_node:main',
        ],
    },
)
