# THU-HKU

ROS2 bicycle balance stack (Raspberry Pi).

## Packages

- `bike_controller` — balance control & launches
- `bike_teleop` — RC / teleop
- `can_motor_controller_MIT` — steer motor (MIT mode)
- `imu_publisher` — IMU
- `sbus_receiver` — SBUS
- `servo_controller_py` — servo / kickstand

## Setup

```bash
git clone git@github.com:Gunner-SJ/THU-HKU.git
cd THU-HKU
# build in your ROS2 workspace as usual
```

Git pull / push: see [`docs/GIT_TUTORIAL.md`](docs/GIT_TUTORIAL.md).

## Launch (examples)

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch bike_controller bike_control_pole.launch.py
ros2 launch bike_controller bike_control_pole_auto_no_rc.launch.py
```
