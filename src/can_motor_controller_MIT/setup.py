from setuptools import find_packages, setup


package_name = "can_motor_controller_mit"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@todo.todo",
    description="GIM3510 dual motor controller with MIT steering control",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "motor_controller_node = can_motor_controller_mit.motor_controller_node:main",
            "steer_timing_test = can_motor_controller_mit.steer_timing_test_ros:main",
        ],
    },
)
