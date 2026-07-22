# bike_balance ROS2 包 — 部署指南

## 原理对照

```
仿真 (notebook)                    真车 (本 ROS2 包)
─────────────────────────────────  ─────────────────────────────────
MuJoCo IMU sensor                  YbIMU → imu_publisher → /imu/data
MuJoCo 编码器 sensor               GIM3510 → can_motor_controller → /motor/state
steer_rate_ref → PI → torque       steer_rate_ref → 积分 → 绝对角度 → set_absolute_position_degrees()
rear_rate_ref → P → torque         rear_rate_ref → set_speed()
MuJoCo mj_step                     GIM3510 内部 2kHz PID (我们不管)
```

**核心差异**: 真车 GIM3510 自带位置/速度/电流三环 (@2kHz)，我们只需发**目标值**，电机自己会追踪。

## 0. 前置要求

树莓派上已有并能正常运行的包:
- `can_motor_controller` (发布 `/motor/state`, 订阅 `/motor/command`)
- `imu_publisher` (发布 `/imu/data`)
- CAN 总线已配置 (`sudo ip link set can0 type can bitrate 1000000; sudo ip link set up can0`)
- IMU 已校准

## 1. 复制到树莓派

```powershell
# Windows 上执行:
scp -r "C:\Users\ASUS_Steven\Desktop\Tsinghua\code\bike_balance_ros2" ubuntu@<树莓派IP>:~/bike_balance_ros2
```

## 2. 树莓派上构建

```bash
ssh ubuntu@<树莓派IP>
cd ~/bike_balance_ros2

# 安装依赖 (如果缺)
pip install numpy scipy

# 构建
colcon build --symlink-install

# Source
source install/setup.bash
echo "source ~/bike_balance_ros2/install/setup.bash" >> ~/.bashrc
```

## 3. 标定 (必须做!)

### 3.1 转向零位标定

```bash
# 1. 先启动 motor node
ros2 run can_motor_controller motor_controller_node

# 2. 把车把摆到正中 (前轮正前方)
# 3. 查看当前角度:
ros2 topic echo /motor/state --once
# 看 position[1] (steer_motor), 将它转为度: rad * 180 / π
# 例如 position[1]=1.524 rad → 87.3°

# 4. 把得到的角度填入 launch/balance.launch.py:
#     'steer_center_deg': 87.3,   ← 改成你的值
```

### 3.2 转向范围标定

```bash
# 车把左转到底 → 读 angle → 计算左边范围
# 车把右转到底 → 读 angle → 计算右边范围
# steer_max_delta_deg = min(左范围, 右范围) - 5° (安全余量)
```

### 3.3 IMU 参数检查

`launch/balance.launch.py` 里已有标定值:
```
'gyro_bias_sensor_dps': [-0.0024, 0.0000, 0.0000],
'sensor_to_bike_rpy_deg': [3.6196, 2.7760, -2.1262]
```
如果重新做了 IMU 标定，更新这些值。

## 4. 符号验证 (最关键! 不做这一步直接跑会倒车)

```bash
cd ~/bike_balance_ros2

# 修改 test_signs.py 顶部的配置 (IMU_PORT, SENSOR_TO_BIKE_RPY_DEG 等)
nano test_signs.py

# 跑测试
python3 test_signs.py
```

**期望结果**:
- 车往右倒 → `u 是负值` (车把左转)
- 车往左倒 → `u 是正值` (车把右转)

如果符号反了 → 改 `SENSOR_TO_BIKE_RPY_DEG` 的 roll +180° (即 [183.6, 2.78, -2.13])

## 5. 运行

### 方式A: 三终端 (无遥控器, 自主平衡)

```bash
# 终端1: IMU
ros2 run imu_publisher imu_node

# 终端2: 电机
ros2 run can_motor_controller motor_controller_node

# 终端3: 平衡
ros2 run bike_balance balance_node
```

### 方式B: 四终端 (带遥控器, 推荐调试用)

```bash
# 终端1: SBUS 遥控接收机 (新增)
ros2 run sbus_receiver sbus_node

# 终端2: IMU
ros2 run imu_publisher imu_node

# 终端3: 电机
ros2 run can_motor_controller motor_controller_node

# 终端4: 平衡 + 遥控
ros2 run bike_balance balance_node
```

### 遥控器操作说明

| 操作 | 遥控器 | 效果 |
|------|--------|------|
| 安全使能 | SWA 拨到 **DOWN** (最下) | 激活遥控, 推油门就走 |
| 急停 | SWA 拨到 MID 或 UP | 电机立即停转 |
| 油门 | 右摇杆 **上下** | 控制前进速度 (0 ~ rc_max_speed) |
| 转向 | 右摇杆 **左右** | 控制 roll 倾角偏移 (车会转弯) |

> **安全机制**: 遥控器断连 0.5 秒后自动急停并回退自主模式。

### 方式C: launch 文件一键启动

### 安全停止
```bash
# 紧急停止 CAN 总线 (电机 1 秒内自动停机):
sudo ip link set can0 down

# Ctrl+C 停止 node
```

## 6. 调参流程

**阶段 4**: 支架撑着, 保守参数, 手扶
```
target_speed = 0.8
lqr_r = 50.0
max_steer_velocity = 2.0
```
→ 观察车把是否有小幅修正动作

**阶段 5**: 逐步放开
```
lqr_r: 50 → 30 → 20 → 10
max_steer_velocity: 2.0 → 3.0 → 5.0
target_speed: 0.8 → 1.0 → 1.5
```

**阶段 6**: 微调
- 晃动太大 → lqr_q_roll_rate 增大 (2→4→6)
- 反应太慢 → lqr_q_roll 增大 (100→200)
- 车把太猛 → lqr_r 增大
- 车把太软 → lqr_r 减小

## 7. 日志分析

Ctrl+C 停止后，CSV 保存在 `~/bike_balance/bike_log_*.csv`。

拷贝回 Windows 画图:
```powershell
scp ubuntu@<IP>:~/bike_balance/bike_log_*.csv "C:\Users\ASUS_Steven\Desktop\Tsinghua\code\"
```

用 notebook 第 7 讲的绘图代码对比仿真 log。

## 文件清单

```
bike_balance_ros2/
├── bike_balance/
│   ├── __init__.py
│   └── balance_node.py       ← 主节点 (平衡算法 + RC 遥控)
├── launch/
│   └── balance.launch.py     ← 启动文件 (含所有参数)
├── calibrate_steer.py        ← 转向零位标定
├── test_signs.py             ← 符号验证脚本 (不依赖ROS2, 可直接跑)
├── resource/
│   └── bike_balance
├── package.xml
├── setup.py
├── setup.cfg
└── DEPLOY.md                 ← 本文档
```
