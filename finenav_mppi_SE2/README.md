# finenav_mppi_SE2

基于模型预测路径积分（Model Predictive Path Integral, MPPI）的地面机器人局部规划器，在 SE(2) 位姿空间（x、y、yaw）中规划，作为 FineNav-Engine 的 planner 插件运行。

## 简介

本规划器改编自 [Nav2 MPPI Controller](https://github.com/ros-navigation/navigation2/tree/main/nav2_mppi_controller)，并在其基础上使用 OpenMP dynamic 调度，对重负载 `CostCritic` 实现了并行计算优化。

MPPI 以机器人当前状态和上一周期的控制序列为起点，叠加高斯噪声采样出一批候选控制序列，前向仿真得到一批轨迹；每条轨迹由一组 critic 打分，再按 softmax 加权得到本周期的最优控制量。

- 运动模型：`DiffDrive` / `Omni` / `Ackermann`
- 包名、共享库名、插件注册键和参数命名空间统一为 `finenav_mppi_SE2`，C++ 插件类型为 `finenav_mppi_SE2::MPPISE2`

## WorldView 要求

WorldView 需实现 `finenav_mppi_SE2::IWorldView`，接口定义见 [world_view_def.hpp](include/finenav_mppi_SE2/world_view_def.hpp)。查询位姿 `(x, y, theta)` 应与机器人状态及参考路径使用同一坐标系，位置单位为米，朝向单位为弧度。

| 接口 | 要求 |
|---|---|
| `getCost(x, y, theta)` | 返回位姿对应的代价值，具体含义由 WorldView 定义 |
| `isCollision(x, y, theta)` | 返回机器人在该位姿是否碰撞，同时用于判断参考路径点是否有效；由 WorldView 统一处理机器人轮廓、未知区域和地图边界的通行策略 |

`CostCritic` 会跳过代价为 0 的位姿的碰撞检测，因此 `getCost()` 的空闲判定需与 `isCollision()` 保持一致。地图边界外的查询也需由 WorldView 妥善处理，避免越界访问。

当 `CostCritic.num_threads > 1` 时，多个线程会同时调用同一 WorldView 的 `getCost()` 和 `isCollision()`，实现必须支持并发只读访问，并与地图更新做好同步。

### 极简示例

```cpp
#include "finenav_mppi_SE2/world_view_def.hpp"

class SimpleWorldView final : public finenav_mppi_SE2::IWorldView {
public:
    int getCost(float x, float y, float theta) const override {
        return isCollision(x, y, theta) ? 100 : 0;
    }

    bool isCollision(float x, float, float) const override {
        return x >= 2.0F;  // 示例：x >= 2 米为障碍区域
    }
};
```

实际使用时，将函数内部替换为地图查询和机器人碰撞检测即可。

## 参数

参数定义在 `src/finenav_mppi_SE2.yaml`，由 `generate_parameter_library` 生成，命名空间为 `finenav_mppi_SE2`。

### 优化器与运动模型

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `time_steps` | int | 56 | 单条轨迹的步数 |
| `model_dt` | double | 0.05 | 预测/控制步长（s），建议取 1 / TrackingPolicy.frequency_hz |
| `batch_size` | int | 2000 | 每轮采样的轨迹数 |
| `iteration_count` | int | 1 | 每个控制周期的优化迭代次数 |
| `temperature` | double | 0.3 | 温度参数，越大越探索，越小越贪心 |
| `gamma` | double | 0.015 | 控制量正则化系数 |
| `motion_model` | string | DiffDrive | DiffDrive / Omni / Ackermann |

### 速度约束

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `vx_max` | double | 0.5 | 最大前进速度（m/s） |
| `vx_min` | double | -0.35 | 最小前进速度（m/s），负值允许倒车 |
| `vy_max` | double | 0.0 | 最大横向速度（m/s），非全向为 0 |
| `wz_max` | double | 1.9 | 最大角速度（rad/s） |

### 加速度约束

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `ax_min` | double | -1.0 | 最小纵向加速度（m/s²） |
| `ax_max` | double | 1.0 | 最大纵向加速度（m/s²） |
| `ay_min` | double | -1.0 | 最小横向加速度（m/s²） |
| `ay_max` | double | 1.0 | 最大横向加速度（m/s²） |
| `az_max` | double | 3.0 | 最大角加速度（rad/s²） |

### 采样噪声

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `vx_std` | double | 0.2 | 纵向速度采样标准差 |
| `vy_std` | double | 0.2 | 横向速度采样标准差 |
| `wz_std` | double | 0.4 | 角速度采样标准差 |
| `regenerate_noises` | bool | true | 每轮优化重新生成噪声 |

### 共享参数

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `threshold_to_consider` | double | 0.5 | 目标类与路径类 critic 之间切换的局部路径长度阈值（m） |

### 可视化

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `visualize` | bool | false | 是否发布轨迹可视化（调试用） |
| `visualizer_frame_id` | string | map | 可视化消息的坐标系 |
| `visualizer_trajectory_topic` | string | /mppi_debug/trajectories | MarkerArray 话题 |
| `visualizer_trajectory_step` | int | 5 | 候选轨迹批量下采样步长 |
| `visualizer_time_step` | int | 3 | 轨迹点时间下采样步长 |
| `visualizer_max_trajectories` | int | 500 | 每帧最多发布的轨迹数 |
| `visualizer_publish_rate` | double | 10.0 | 发布频率上限（Hz） |

### Critic

下面每个 critic 都列出全部参数。公共参数为 `enabled`（bool，是否启用）、`cost_power`（int，代价幂次，默认 1）、`cost_weight`（double，权重）。

#### ConstraintCritic — 惩罚违反运动学/加速度约束的轨迹

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `enabled` | bool | true | 是否启用 |
| `cost_power` | int | 1 | 代价幂次 |
| `cost_weight` | double | 4.0 | 权重 |

#### GoalCritic — 拉向目标点

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `enabled` | bool | true | 是否启用 |
| `cost_power` | int | 1 | 代价幂次 |
| `cost_weight` | double | 5.0 | 权重 |

#### GoalAngleCritic — 末态朝向对齐目标朝向

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `enabled` | bool | true | 是否启用 |
| `cost_power` | int | 1 | 代价幂次 |
| `cost_weight` | double | 3.0 | 权重 |

#### PreferForwardCritic — 偏向前进而非倒车

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `enabled` | bool | true | 是否启用 |
| `cost_power` | int | 1 | 代价幂次 |
| `cost_weight` | double | 5.0 | 权重 |

#### CostCritic — 用代价栅格避障

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `enabled` | bool | true | 是否启用 |
| `cost_power` | int | 1 | 代价幂次 |
| `cost_weight` | double | 3.81 | 权重（内部按 254 归一化） |
| `num_threads` | int | 1 | OpenMP dynamic 调度的 worker 线程数，1 为串行 |
| `chunk_size` | int | 1 | 每次动态领取的轨迹数 |
| `critical_cost` | double | 300.0 | 位姿代价 ≥ 253（近致命）时的惩罚 |
| `collision_cost` | double | 1000000.0 | 轨迹碰撞时的惩罚 |
| `near_goal_distance` | double | 1.0 | 距目标此距离内关闭排斥（m） |
| `consider_footprint` | bool | true | 用完整 footprint 碰撞检测而非中心点 |

#### PathAlignCritic — 轨迹贴近参考路径

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `enabled` | bool | true | 是否启用 |
| `cost_power` | int | 1 | 代价幂次 |
| `cost_weight` | double | 14.0 | 权重 |
| `max_path_occupancy_ratio` | double | 0.05 | 路径被占用比例超过该值时禁用此 critic |
| `trajectory_point_step` | int | 4 | 对齐评估时轨迹点的采样步长 |
| `offset_from_furthest` | int | 20 | 相对最远可达路径点的偏移 |
| `use_path_orientations` | bool | false | 是否把路径点朝向纳入对齐代价 |

#### PathFollowCritic — 轨迹端点落在参考路径前方

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `enabled` | bool | true | 是否启用 |
| `cost_power` | int | 1 | 代价幂次 |
| `cost_weight` | double | 5.0 | 权重 |
| `offset_from_furthest` | int | 6 | 相对最远可达路径点的偏移 |

#### PathAngleCritic — 朝向对齐参考路径方向

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `enabled` | bool | true | 是否启用 |
| `cost_power` | int | 1 | 代价幂次 |
| `cost_weight` | double | 2.0 | 权重 |
| `offset_from_furthest` | int | 4 | 相对最远可达路径点的偏移 |
| `max_angle_to_furthest` | double | 1.0 | 到最远路径点的最大允许偏航误差（rad） |

#### TwirlingCritic — 惩罚原地打转（默认关闭）

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `enabled` | bool | false | 是否启用 |
| `cost_power` | int | 1 | 代价幂次 |
| `cost_weight` | double | 10.0 | 权重 |

#### VelocityDeadbandCritic — 惩罚死区内的低速（默认关闭）

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `enabled` | bool | false | 是否启用 |
| `cost_power` | int | 1 | 代价幂次 |
| `cost_weight` | double | 35.0 | 权重 |
| `deadband_velocities` | double[] | [0.05, 0.05, 0.05] | 死区阈值 [vx, vy, wz]，低于该值被惩罚 |

## 示例

参数通过 ROS 2 参数文件加载，以 `finenav_mppi_SE2` 为前缀声明。下面是一份使用全部默认值的完整配置：

```yaml
finenav_mppi_SE2:
  # 优化器
  time_steps: 56
  model_dt: 0.05
  batch_size: 2000
  iteration_count: 1
  temperature: 0.3
  gamma: 0.015
  motion_model: "DiffDrive"

  # 速度
  vx_max: 0.5
  vx_min: -0.35
  vy_max: 0.0
  wz_max: 1.9

  # 加速度
  ax_min: -1.0
  ax_max: 1.0
  ay_min: -1.0
  ay_max: 1.0
  az_max: 3.0

  # 采样
  vx_std: 0.2
  vy_std: 0.2
  wz_std: 0.4
  regenerate_noises: true

  # 共享
  threshold_to_consider: 0.5

  # 可视化（调试时改为 true）
  visualize: false
  visualizer_frame_id: "map"
  visualizer_trajectory_topic: "/mppi_debug/trajectories"
  visualizer_trajectory_step: 5
  visualizer_time_step: 3
  visualizer_max_trajectories: 500
  visualizer_publish_rate: 10.0

  # critic
  ConstraintCritic:
    enabled: true
    cost_power: 1
    cost_weight: 4.0
  GoalCritic:
    enabled: true
    cost_power: 1
    cost_weight: 5.0
  GoalAngleCritic:
    enabled: true
    cost_power: 1
    cost_weight: 3.0
  PreferForwardCritic:
    enabled: true
    cost_power: 1
    cost_weight: 5.0
  CostCritic:
    enabled: true
    cost_power: 1
    cost_weight: 3.81
    num_threads: 1
    chunk_size: 1
    critical_cost: 300.0
    collision_cost: 1000000.0
    near_goal_distance: 1.0
    consider_footprint: true
  PathAlignCritic:
    enabled: true
    cost_power: 1
    cost_weight: 14.0
    max_path_occupancy_ratio: 0.05
    trajectory_point_step: 4
    offset_from_furthest: 20
    use_path_orientations: false
  PathFollowCritic:
    enabled: true
    cost_power: 1
    cost_weight: 5.0
    offset_from_furthest: 6
  PathAngleCritic:
    enabled: true
    cost_power: 1
    cost_weight: 2.0
    offset_from_furthest: 4
    max_angle_to_furthest: 1.0
  TwirlingCritic:
    enabled: false
    cost_power: 1
    cost_weight: 10.0
  VelocityDeadbandCritic:
    enabled: false
    cost_power: 1
    cost_weight: 35.0
    deadband_velocities: [0.05, 0.05, 0.05]
```

上面的值即 `src/finenav_mppi_SE2.yaml` 里的默认值；实际使用中只需覆写要改的参数。
