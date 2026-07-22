# can_motor_controller_mit

ROS 2 Jazzy package for two GIM3510 motors on one CAN bus:

- Motor ID 1: standard `0xC1` speed control.
- Motor ID 2: MIT 8-byte position/velocity/Kp/Kd/torque control.
- `/motor/command`: `sensor_msgs/msg/JointState` command input.
- `/motor/state`: `sensor_msgs/msg/JointState` feedback output.

The controller node is the only process that may open `can0`. Stop the old
`motor_controller_node`, balance controller, and standalone CAN test scripts
before the first MIT test.

## 1. Copy and build

Copy this whole folder to:

```text
/home/ubuntu/ws_ros2/src/can_motor_controller_MIT
```

Then run:

```bash
cd ~/ws_ros2
sudo apt update
sudo apt install -y python3-can python3-matplotlib
colcon build --packages-select can_motor_controller_mit --symlink-install
source /opt/ros/jazzy/setup.bash
source ~/ws_ros2/install/setup.bash
```

## 2. Safety checks

Raise and secure the front wheel. Keep hands, cables, and tools outside the
steering range. Be ready to cut motor power.

The default MIT test range is only `90 +/- 3 degrees`. Do not increase it until
the reported steering angle is correct and the small-angle test succeeds.

The host uses the protocol-default MIT limits:

```text
Pos_Max = 95.5 rad
Vel_Max = 45.0 rad/s
T_Max   = 18.0 Nm
```

These values must match the `0xF0` values stored in motor ID 2. The package does
not overwrite non-volatile `0xF0` values by default.

## 3. Start the MIT controller

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_ros2/install/setup.bash
ros2 run can_motor_controller_mit motor_controller_node
```

Expected startup text includes:

```text
MIT Motor Node started: drive ID=1 (C1), steer ID=2 (MIT)
CAN opened; ID1 C1 and ID2 MIT are ready
```

Check that no other node publishes motor commands:

```bash
ros2 topic info /motor/command -v
```

Before starting the timing test, publisher count should be zero.

Verify steering feedback before commanding motion:

```bash
ros2 topic echo /motor/state --once
```

At the physical steering center, `steer_motor` position should be close to
`1.5708 rad` (90 degrees). Stop if the MIT position is not consistent with the
physical steering angle.

## 4. Run one timing test

Edit these values in
`can_motor_controller_mit/steer_timing_test_ros.py`:

```python
MIT_KP = 5.0
MIT_KD = 0.2
MAX_DELTA_DEG = 3.0
```

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_ros2/install/setup.bash
ros2 run can_motor_controller_mit steer_timing_test
```

Results are written to:

```text
~/steer_test_results/
```

Each run saves a CSV log and a PNG containing target/actual angle, error,
velocity, and MIT torque.

## 5. Confirm MIT traffic

Use a third terminal:

```bash
candump -L can0,502:7FF,002:7FF
```

MIT commands for motor ID 2 should appear as `502#` with eight data bytes. A
normal immediate response is an ID `002` seven-byte status frame beginning with
`F1`.

## 6. Initial tuning order

1. Keep `MIT_KD = 0.2` and test `MIT_KP = 5`.
2. Increase KP one step at a time: `5, 10, 20, 30`.
3. Compare median arrival time, overshoot, peak torque, and repeatability.
4. If overshoot or impact increases, raise KD gradually: `0.2, 0.3, 0.5`.
5. Increase `MAX_DELTA_DEG` only after the 3-degree test is repeatable.

`MIT_KP` is encoded over `[0, 500]`; `MIT_KD` is encoded over `[0, 5]` as
specified by protocol version 3.09b0.
