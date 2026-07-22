"""Distinct-pole + no-RC AUTO. Kickstands: speed-only via kickstand_speed."""

import os
import subprocess
import time

from launch import LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node

LOG_DIR = os.path.expanduser("~/ws_ros2/bike_response_logs")

KS_PARAMS = {
    'wheel_radius': 0.0725,
    'kickstand_speed_deploy': 1.2,
    'kickstand_speed_retract': 1.5,
    'kickstand_lean_bias_rad': 0.0,
    'servo_center_rad_1': 2.82,
    'servo_center_rad_2': 0.72,
    'servo_max_swing_rad': 0.7854,
    'servo_cmd_topic': '/servo/command',
}


def _plot_on_shutdown(context):
    time.sleep(0.5)
    os.makedirs(LOG_DIR, exist_ok=True)
    print(f"[pole launch] plotting + opening latest CSV in {LOG_DIR} ...")
    subprocess.run(
        [
            "ros2", "run", "bike_controller", "plot_bike_log",
            "--log-dir", LOG_DIR, "--latest", "--open",
        ],
        check=False,
    )
    return []


def generate_launch_description():
    os.makedirs(LOG_DIR, exist_ok=True)

    return LaunchDescription([
        Node(
            package='can_motor_controller_mit',
            executable='motor_controller_node',
            name='motor_controller_node',
            output='screen',
            parameters=[{
                'steer_center_deg': 0.0,
                'steer_max_delta_deg': 24.50,
                'steer_max_left_deg': 24.50,
                'steer_max_right_deg': 30.00,
                'drive_watchdog_sec': 2.0,
                'home_on_start': True,
                'home_on_reconnect': False,
                'startup_home_position_deg': 0.0,
                'startup_home_tolerance_deg': 2.0,
                'startup_home_speed_limit_rpm': 100.0,
                'startup_home_current_limit_a': 6.0,
                'startup_home_timeout_sec': 8.0,
            }]
        ),

        Node(
            package='servo_controller_py',
            executable='servo_node',
            name='servo_node',
            output='screen'
        ),

        Node(
            package='bike_controller',
            executable='kickstand_speed',
            name='kickstand_speed',
            output='screen',
            parameters=[KS_PARAMS],
        ),

        Node(
            package='imu_publisher',
            executable='imu_node',
            name='imu_publisher',
            output='screen'
        ),

        Node(
            package='bike_teleop',
            executable='force_auto_ref',
            name='force_auto_ref',
            output='screen'
        ),

        Node(
            package='bike_controller',
            executable='balance_executor',
            name='bike_balance_executor',
            output='screen',
            parameters=[{
                'control_method': 'place_distinct_poles',
                # Max-aggression recovery test (full hardware steer authority).
                'poles_ctr': [-12.0, -24.0, -40.0],
                'pole_wc': -5.0,
                'control_dt': 0.02,
                'log_dir': LOG_DIR,
                'wheel_radius': 0.0725,
                'target_speed': 1.5,
                'acceleration_time': 1.0,
                'min_scheduling_speed': 0.5,
                'initial_steer_deg': -10.0,
                'drive_sign': -1.0,
                'max_steer_velocity': 12.0,
                # Soft cap OFF. Unwind OFF for recovery test — forced center pull
                # was blocking hold of full-right into-the-fall (“只能打到中间”).
                'sat_unwind_enable': False,
                'sat_unwind_roll_deg': 8.0,
                'sat_unwind_rate_rad_s': 4.0,
                'sat_steer_roll_gain': 0.0,
                'sat_steer_floor_deg': 8.0,
                'lqr_q_steer': 12.0,
                'lqr_q_roll': 120.0,
                'lqr_q_roll_rate': 12.0,
                'lqr_r_steer_rate': 25.0,
                'steer_center_deg': -6.0,
                'steer_max_delta_deg': 24.50,
                'steer_max_left_deg': 24.50,
                'steer_max_right_deg': 30.00,
                'roll_limit_deg': 40.0,
                'flip_roll_sign': False,
                'gyro_bias_sensor_dps': [-0.0000, 0.0000, 0.0000],
                'sensor_to_bike_rpy_deg': [3.6196, 2.7760, -2.1262],
            }]
        ),

        RegisterEventHandler(
            OnShutdown(on_shutdown=[OpaqueFunction(function=_plot_on_shutdown)])
        ),
    ])
