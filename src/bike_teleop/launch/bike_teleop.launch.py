import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. SBUS RC Receiver
        Node(
            package='sbus_receiver',
            executable='sbus_node',
            name='sbus_node',
            output='screen'
        ),
        
        # # 2. Rear Drive Motor (GIM3510-8)
        # Node(
        #     package='rear_servo',
        #     executable='rear_servo_node',  # Changed from 'motor_node'
        #     name='rear_servo',
        #     output='screen',
        #     parameters=[{
        #         'can_channel': 'can0',
        #         'rear_addr': 1,
        #         'publish_rate': 50.0,
        #         'cmd_timeout': 0.2,
        #     }],
        # ),
        
        # # 3. Steering Motor (GIM3510-8)
        # Node(
        #     package='steer_servo',
        #     executable='steer_servo_node',  # Changed from 'motor_node' (assuming same pattern)
        #     name='steer_servo',
        #     output='screen',
        #     parameters=[{
        #         'can_channel': 'can0',
        #         'steer_addr': 2,  # Adjust address as needed
        #         'publish_rate': 50.0,
        #         'cmd_timeout': 0.2,
        #     }],
        # ),

                # 3. can motor controller
        Node(
            package='can_motor_controller',
            executable='motor_controller_node',
            name='motor_controller_node',
            output='screen'
        ),
        
        # 4. UART Servos (STS3250)
        Node(
            package='servo_controller_py',
            executable='servo_node',
            name='servo_node',
            output='screen'
        ),
        
        # 5. IMU Sensor
        Node(
            package='imu_publisher',
            executable='imu_node',
            name='imu_publisher',
            output='screen'
        ),
        
        # 6. Teleop Bridge (Your main control logic)
        Node(
            package='bike_teleop',
            executable='teleop_node',
            name='teleop_node',
            output='screen'
        ),
    ])