from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bike_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Bike teleop / RC supervisor',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_supervisor = bike_teleop.teleop_supervisor:main',
            'force_auto_ref = bike_teleop.force_auto_ref:main',
            'teleop_node = bike_teleop.teleop_node:main',
            'servo_calibrator = bike_teleop.servo_calibrator:main',
        ],
    },
)
