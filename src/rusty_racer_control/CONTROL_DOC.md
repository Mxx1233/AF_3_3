# Rusty Racer 控制系统文档

> **版本**：V7（2026-02-27）
> **作者**：Rusty Racer Team
> **状态机版本**：F2

---

## 1. 系统架构总览

```
感知层                        控制层                       执行层
┌──────────────────┐         ┌──────────────────────────┐  ┌─────────────┐
│ /traffic_sign/   │──────→  │                          │  │             │
│   detections     │         │      ControlNode         │  │  /motor_    │
│                  │         │   (laneCallback @50Hz)   │→ │   command   │
│ /lane_deviation  │──────→  │                          │  │             │
│                  │         │  纵向：开环前馈           │  │ motor_level │
│ /odom            │──────→  │  横向：PD控制器           │  │ steer_angle │
└──────────────────┘         └──────────────────────────┘  └─────────────┘
```

---

## 2. ROS2 话题与通信

| 话题 | 类型 | 方向 | 频率 | 说明 |
|------|------|------|------|------|
| `/lane_deviation` | `LaneDeviation` | 订阅 | ~50 Hz | 主控制循环触发源 |
| `/odom` | `Odometry` | 订阅 | ~50 Hz | 当前速度（odom低通滤波）|
| `/traffic_sign/detections` | `TrafficSign` | 订阅 | 异步 | 交通标志识别结果 |
| `/motor_command` | `MotorCommand` | 发布 | ~50 Hz | 电机油门 + 转向角 |

**交通标志超时**：若超过 **2.5 秒**未收到标志消息，FSM 以空帧运行（无标志输入）。

---

## 3. 控制模式选择

通过 ROS2 参数 `cruise_mode` 在启动时选择：

```bash
# 模式一：巡航模式（仅启动门）
ros2 run rusty_racer_control control_node --ros-args -p cruise_mode:=true

# 模式二：完整 FSM 模式（默认）
ros2 run rusty_racer_control control_node --ros-args -p cruise_mode:=false
```

| 参数值 | 模式名称 | 速度来源 | 适用场景 |
|--------|----------|----------|----------|
| `true` | Cruise（巡航）| Stop 牌启动门 → 固定 v_ref 巡航 | 简单测速、调试 |
| `false` | Full FSM | 三层状态机根据标志动态规划 | 比赛正式运行 |

---

## 4. 纵向速度控制（V7 开环）

### 4.1 控制公式

```
motor_level = clip( v_target / v_max, 0.0, 1.0 )
```

- `v_target`：由模式决定的目标速度 [m/s]
- `v_max = 2.0 m/s`：电机归一化基准（需与实际最高速对应）
- `motor_level ∈ [0.0, 1.0]`：最终发布的电机指令

### 4.2 开环 vs 闭环（设计演进）

| 特性 | V6（位置式 PI） | V7（开环前馈）|
|------|----------------|--------------|
| 控制方式 | 闭环（反馈 odom 速度）| 开环（纯前馈映射）|
| 积分问题 | integral_ss=38.3，存在过渡期加速 | 无积分，无过渡期 |
| 启动行为 | motor=43.1%，可能低于静摩擦 | motor=57.5%，直接目标油门 |
| 速度精度 | 理论精确跟踪 v_ref | 依赖 v_max 校准 |
| 参数复杂度 | 需调 Kp、Ki | 只需校准 v_max |

### 4.3 各速度区 motor_level 映射

| 速度区 | v_target [m/s] | motor_level | 说明 |
|--------|---------------|-------------|------|
| Default（巡航）| v_ref（动态）| v_ref / 2.0 | 默认行驶速度，可运行时调整 |
| Speed30（限速区）| 0.75 | 37.5% | 限速 30 区减速 |
| Highway（高速区）| 1.5 | 75.0% | 高速道路加速 |
| Yield（让行）| 0.3 | 15.0% | 接近 Yield 标志减速 |
| Stop 触发 | 0.0 | 0% | 完全停车 |

### 4.4 运行时速度调整

```bash
# 修改巡航/Default 区速度（立即生效，无需重启）
ros2 param set /control_node v_ref 1.3

# 查看当前值
ros2 param get /control_node v_ref
```

**影响范围**：
- `cruise_mode=true`：修改软启动斜坡的目标速度
- `cruise_mode=false`：修改 FSM Default 区速度（Speed30/Highway/Yield 不变）

---

## 5. 横向控制（PD 控制器）

### 5.1 控制结构

```
转向角 δ = Kp_lat × y_error + Kd_lat × heading_error
```

- `y_error`：车道偏差（来自 `/lane_deviation`）[m]
- `heading_error`：航向角偏差 [rad]
- 最终指令取反：`steering_angle = -δ`（坐标系修正）

### 5.2 横向参数

| 参数 | 值 | 说明 |
|------|----|------|
| `lat_kp` | 0.75 | 比例增益 |
| `lat_kd` | 0.15 | 微分增益（航向角） |
| `y_target_` | -0.06 m | 目标横向偏置（左偏 6cm）|
| 轴距 `l` | 0.257 m | 自行车模型轴距 |

### 5.3 速度自适应

横向控制器通过 `odomCallback` 实时更新当前速度（`updateVelocity`），调整转向响应幅度。

---

## 6. 交通标志状态机（FSM F2）

### 6.1 三层架构

```
输入：CamFrame（交通标志检测 + 距离）
         ↓
┌─────────────────────────────────┐
│  Layer 0：启动门（Start Gate）  │  → 阻止车辆在赛事开始前行驶
│  Stop 牌出现 → 消失 → 解锁     │
└─────────────────────────────────┘
         ↓（门解锁后）
┌─────────────────────────────────┐
│  Layer 1：速度模式切换          │  → 决定当前速度区
│  Default ↔ Speed30 ↔ Highway   │  → 带延时 commit 策略
└─────────────────────────────────┘
         ↓
┌─────────────────────────────────┐
│  Layer 2：动作触发              │  → 优先级最高，直接覆盖速度
│  Stop 停车保持 / Yield 减速     │
└─────────────────────────────────┘
         ↓
输出：DecisionOut { v_ref_mps, must_stop }
```

### 6.2 识别的标志类型

| 标志 ID（字符串）| 类型 | 触发效果 |
|-----------------|------|----------|
| `stop_sign` | Stop | Layer 0 启动门 + Layer 2 停车 |
| `start_zone_speed_limit` | Speed30Start | Layer 1：进入限速区 |
| `end_zone_speed_limit` | Speed30End | Layer 1：退出限速区 |
| `start_express_way` | HighwayStart | Layer 1：进入高速区 |
| `end_express_way` | HighwayEnd | Layer 1：退出高速区 |
| `yield` | YieldSlow | Layer 2：减速让行 |

### 6.3 Layer 0：启动门逻辑

两阶段确认，防误触发：

```
Phase 1：等待 Stop 牌出现
  - 连续 start_on_count = 3 帧检测到 Stop 牌 → 确认存在
Phase 2：等待 Stop 牌消失（裁判员拿走）
  - 连续 start_off_count = 6 帧未检测到 Stop 牌 → 门解锁，开始行驶
```

**两种模式的启动门对比**：

| 特性 | cruise_mode=true | cruise_mode=false（FSM）|
|------|------------------|-------------------------|
| 实现位置 | control_node.cpp | traffic_fsm2.h |
| 解锁后 | 以 0.3 m/s² 斜坡加速到 v_ref | 直接输出 v_default |
| PI 重置 | 解锁时重置 integral | 无 PI，无需重置 |

### 6.4 Layer 1：速度模式切换（Commit 策略）

检测到速度区标志后**不立即切换**，而是等待延时结束再切换，防止感知抖动引起频繁切换：

```
检测到 Speed30Start → on_count 帧确认 → 启动 s30_start_delay_ms 计时器
计时器到期 → 实际切换到 Speed30 模式
```

**冷却逻辑**：Stop/Yield 触发后设置冷却标志，解除条件：
```cpp
if (!has_stop || stop_dist >= d_release)  // 牌子消失 OR 距离 ≥ 3m
    stop_cooldown_ = false;
```

### 6.5 Layer 2：Stop 触发与保持

```
触发条件：stop_armed_ && !stop_cooldown_
         && has_stop && stop_dist ≤ d_stop_trigger（1.0m）
         && 连续 on_count（3）帧满足

触发后：
  - must_stop = true → v_target = 0 → motor = 0%
  - 保持 stop_hold_ms = 3000 ms
  - 保持期结束 → stop_cooldown_ = true → 恢复行驶
```

---

## 7. 传感器处理

### 7.1 odom 速度低通滤波

```cpp
alpha = 0.5
current_v_ = alpha × raw_v + (1 - alpha) × current_v_
```

等效截止频率：约 **8 Hz**（50Hz 采样，α=0.5，时间常数 ≈ 20ms）

### 7.2 交通标志预处理

- 有效距离范围：`[cam_min_valid_dist_m, cam_max_valid_dist_m] = [0.30m, 6.00m]`
- 同一标志在 1.0m 内不重复触发（`same_sign_block_dist_m`）
- 以**最近的**同类标志距离参与计算

---

## 8. 完整参数表

### 8.1 纵向速度参数

| 参数 | 位置 | 当前值 | 修改方式 | 说明 |
|------|------|--------|----------|------|
| `v_ref` | ROS2 参数 | 1.15 m/s | `ros2 param set` | Default 区巡航速度 |
| `v_max` | L68 构造函数 | 2.0 m/s | 改代码重编译 | 电机归一化基准 |
| `v_speed30` | L76 构造函数 | 0.75 m/s | 改代码重编译 | 限速区目标速度 |
| `v_highway` | L77 构造函数 | 1.5 m/s | 改代码重编译 | 高速区目标速度 |
| `v_yield` | L78 构造函数 | 0.3 m/s | 改代码重编译 | 让行速度 |

### 8.2 横向控制参数

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `lat_kp` | 0.75 | 横向误差比例增益 |
| `lat_kd` | 0.15 | 航向角微分增益 |
| `y_target_` | -0.06 m | 期望横向偏置（负值=左偏）|
| 轴距 `l` | 0.257 m | 车辆轴距 |

### 8.3 FSM 触发距离参数

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `d_stop_trigger` | 1.0 m | Stop 触发距离 |
| `d_yield_trigger` | 2.0 m | Yield 触发距离 |
| `d_release` | 3.0 m | 冷却解除距离 |
| `stop_hold_ms` | 3000 ms | Stop 保持时长 |

### 8.4 FSM 去抖参数

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `on_count` | 3 帧 | 标志确认帧数（Layer 1/2）|
| `off_count` | 3 帧 | 标志消失确认帧数 |
| `start_on_count` | 3 帧 | 启动门 Phase 1 确认帧数 |
| `start_off_count` | 6 帧 | 启动门 Phase 2 消失帧数 |
| `cam_min_valid_dist_m` | 0.30 m | 标志最近有效距离 |
| `cam_max_valid_dist_m` | 6.00 m | 标志最远有效距离 |

### 8.5 Layer 1 速度切换延时参数

延时公式：`delay_ms ≈ 检测距离(~2m) / 接近速度(m/s) × 1000`

| 参数 | 当前值 | 接近速度基准 | 说明 |
|------|--------|-------------|------|
| `s30_start_delay_ms` | 1500 ms | ~1.0 m/s | 进入限速区的延时 |
| `s30_end_delay_ms` | 3000 ms | ~0.5 m/s | 退出限速区的延时 |
| `hw_start_delay_ms` | 1500 ms | ~1.0 m/s | 进入高速区的延时 |
| `hw_end_delay_ms` | 1000 ms | ~1.5 m/s | 退出高速区的延时 |

---

## 9. 数据流图（laneCallback 一帧执行流程）

```
laneCallback 触发（~50Hz）
    │
    ├── 计算 dt（时间步长，默认 0.02s）
    │
    ├── 读取动态参数 v_ref（ros2 param）
    ├── 同步 traffic_params_.v_default = v_ref
    │
    ├── [cruise_mode=true]  巡航路径
    │     ├── gate 未解锁 → v_target=0, emergency_stop=true
    │     └── gate 已解锁 → v_ramp 以 0.3m/s² 斜坡爬升到 v_ref → v_target=v_ramp
    │
    ├── [cruise_mode=false] FSM 路径
    │     ├── 检查标志消息年龄（超过 2.5s 用空帧）
    │     └── traffic_fsm_.step() → v_target, emergency_stop
    │           ├── Layer 0：gate 锁定 → must_stop=true
    │           ├── Layer 1：速度模式 → v_ref_mps（Default/S30/Highway）
    │           └── Layer 2：Stop/Yield → 覆盖 must_stop, v_ref_mps
    │
    ├── emergency_stop=true → v_target=0
    │
    ├── 纵向计算（V7 开环）
    │     motor_level = clip(v_target / v_max, 0.0, 1.0)
    │
    ├── 横向计算（PD）
    │     delta = lat_kp × y_error + lat_kd × heading_error
    │
    └── 发布 /motor_command { motor_level, steering_angle=-delta }
```

---

## 10. 版本演进摘要

| 版本 | 速度控制 | 关键改进 | 遗留问题 |
|------|----------|----------|----------|
| V1-V3 | 增量式 PI + decay | 基础功能 | 振荡、双重滞后 |
| V4 | 增量式 PI + 前馈 | 去电机平滑 | 启动冲击 |
| V5 | 增量式 PI + 斜坡启动 | 软启动 | 越开越快 |
| V6 | **位置式 PI** + anti-windup | 速度卡顿消除 | 积分过渡期加速 |
| **V7** | **开环前馈**（当前）| 无积分、无超调 | 需校准 v_max |

| 版本 | FSM 改进 | 效果 |
|------|----------|------|
| F1 | 冷却逻辑 AND→OR | Stop 牌每次均触发 ✅ |
| **F2** | Layer 1 加入 pending 延时计时器 | 速度切换时机对齐标志位置 ✅ |

---

## 11. 常用调试命令

```bash
# 查看当前所有参数
ros2 param list /control_node
ros2 param get /control_node v_ref

# 运行时调整巡航速度
ros2 param set /control_node v_ref 1.3

# 切换控制模式（需重启节点）
ros2 param set /control_node cruise_mode true

# 监听电机指令输出
ros2 topic echo /motor_command

# 监听标志检测
ros2 topic echo /traffic_sign/detections

# 查看 FSM 日志
ros2 topic echo /rosout | grep control_node
```
