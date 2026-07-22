"""
bike_balance launch — 只启动平衡控制节点
(配合已有的 can_motor_controller + imu_publisher)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bike_balance_ros2',
            executable='balance_node',
            name='bike_balance_node',
            output='screen',
            parameters=[{
                # ═══════════════════════════════════════════════════
                # 控制方法: "lqr" 或 "pole"
                # ═══════════════════════════════════════════════════
                'control_method': 'lqr',
                'control_dt': 0.02,

                # ═══════════════════════════════════════════════════
                # 物理参数 (已实测) — 通常不需要改
                # ═══════════════════════════════════════════════════
                'rear_contact_to_com': 0.117,
                'wheelbase': 0.28223,
                'trail': 0.0140,
                'com_height': 0.105,
                'mass': 2.218,
                'roll_inertia': 0.02445,
                'steering_axis_angle': 0.179770,

                # ═══════════════════════════════════════════════════
                # 速度控制
                # ═══════════════════════════════════════════════════
                'wheel_radius': 0.0725,
                'target_speed': 0.8,             # 保守起步: 0.8 m/s
                'acceleration_time': 3.0,
                'min_scheduling_speed': 0.5,

                # ═══════════════════════════════════════════════════
                # LQR 权重 (阶段4保守值 → 阶段6逐步放开)
                # ═══════════════════════════════════════════════════
                'lqr_q_steer': 4.0,
                'lqr_q_roll': 100.0,
                'lqr_q_roll_rate': 2.0,
                'lqr_r': 50.0,                   # 保守: 50 → 正常: 10
                'max_steer_velocity': 2.0,       # 保守: 2.0 → 正常: 5.0

                # 极点配置 (method="pole" 时才用)
                'pole_wc': -5.0,

                # ═══════════════════════════════════════════════════
                # 转向零位 & 范围 【⚠️ 必须标定!】
                # ═══════════════════════════════════════════════════
                'steer_center_deg': 90.0,         # ← 车把正中时的电机绝对角度
                'steer_max_delta_deg': 20.0,      # ← 单侧最大偏离角度

                # ═══════════════════════════════════════════════════
                # IMU 标定 【⚠️ 必须标定!】
                # ═══════════════════════════════════════════════════
                'gyro_bias_sensor_dps': [-0.0024, 0.0000, 0.0000],
                'sensor_to_bike_rpy_deg': [3.6196, 2.7760, -2.1262],

                # ═══════════════════════════════════════════════════
                # ROS2 Topic 名称
                # ═══════════════════════════════════════════════════
                'imu_topic': '/imu/data',
                'motor_state_topic': '/motor/state',
                'motor_cmd_topic': '/motor/command',

                # ═══════════════════════════════════════════════════
                # 安全
                # ═══════════════════════════════════════════════════
                'roll_limit_deg': 20.0,
                'start_balance_after_s': 0.5,

                # ═══════════════════════════════════════════════════
                # RC 遥控 (可选, 需配合 sbus_receiver 包)
                #   SWA DOWN=激活, 右摇杆垂直=油门, 右摇杆水平=转向
                # ═══════════════════════════════════════════════════
                'rc_enabled': True,
                'rc_max_speed': 1.5,             # 油门推到顶的速度 (保守起步)
                'rc_max_roll_rad': 0.175,        # ≈ 10° roll 偏移
                'rc_timeout': 0.5,               # RC 信号超时回退 (秒)
                'rc_throttle_deadzone': 0.1,    # 油门死区, ±10% 内视为回中

                # ═══════════════════════════════════════════════════
                # 日志
                # ═══════════════════════════════════════════════════
                'log_dir': '/home/ubuntu/bike_balance/',
            }],
        ),
    ])
