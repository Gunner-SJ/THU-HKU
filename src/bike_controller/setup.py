from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bike_controller'

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
    description='Bicycle balance controller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'balance_executor = bike_controller.balance_executor:main',
            'balance_node = bike_controller.balance_node:main',
            'kickstand_speed = bike_controller.kickstand_speed_node:main',
            'plot_bike_log = bike_controller.plot_bike_log:main',
        ],
    },
)
