from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bike_balance_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Steven',
    maintainer_email='steven@bike.local',
    description='50Hz bicycle balance controller',
    license='MIT',
    entry_points={
        'console_scripts': [
            'balance_node = bike_balance.balance_node:main',
        ],
    },
)
