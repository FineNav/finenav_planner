# finenav_route_planner

栅格路径规划器集合，作为 FineNav-Engine 的 planner 插件运行，包含四个 planner：

- `AstarPathSearch` — 均匀栅格 A* 搜索
- `BfsPathSearch` — 无方向启发式的栅格搜索（A* + Heuristic::Zero，即 Dijkstra / 加权 BFS）
- `DStarLitePathSearch` — 有状态的 D* Lite 增量搜索，地图局部变化时仅重算受影响区域
- `DetourPlanner` — 局部绕障规划器，检测参考轨迹上的堵塞并生成栅格绕行路径

## 简介

四个 planner 都实现 FineNav 的 `Planner` 概念：声明 `IWorldView` 读取地图，`plan(ctx, world_view)` 返回轨迹。

- `AstarPathSearch` / `BfsPathSearch` 共用 `astar::IWorldView`（`isWalkable` / `getCost` / `getStepSize`），在均匀栅格上做搜索；BFS 等价于启发函数为零的 A*，均匀地图上即最少跳数路径
- `DStarLitePathSearch` 使用专属的 `IWorldView`（在 `astar::IWorldView` 基础上扩展 `rows()` / `cols()` / `layers()` 栅格维度查询），在多次规划之间维护 g/rhs 值与优先队列，障碍物变化时做增量重规划
- `DetourPlanner` 使用 `astar::IWorldView`，位于全局路线规划器与 MPPI 之间：输出路径始终从机器人当前位置出发，用障碍物感知的搜索重新挂接到参考轨迹上，并绕过被堵塞的路段
- 栅格步长由 `IWorldView::getStepSize()` 在规划时提供，不是参数

## 参数

四个 planner 参数命名空间不同，参数分别定义在 `astar.yaml`、`bfs.yaml`、`dstarlite.yaml`、`detour.yaml`。

### AstarPathSearch（命名空间 `astar_planner`）

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `connectivity` | string | "8" | 栅格连通性：4/8（2D），6/18/26（3D） |
| `heuristic` | string | "Euclidean" | 启发函数：Euclidean / Manhattan / Chebyshev |
| `goal_tolerance` | double | 0.5 | 到目标的接受距离（m） |
| `max_iterations` | int | 1000000 | 最大扩展节点数（-1 不限制） |
| `max_planning_time` | double | 5.0 | 最大规划时长（s） |
| `movement_weight` | double | 1.0 | A* 评估函数中移动代价的权重 |
| `greedy_weight` | double | 1.001 | 贪心启发项的权重 |

### BfsPathSearch（命名空间 `bfs_planner`）

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `connectivity` | string | "8" | 栅格连通性：4/8（2D），6/18/26（3D） |
| `goal_tolerance` | double | 0.5 | 到目标的接受距离（m） |
| `max_iterations` | int | 1000000 | 最大扩展节点数（-1 不限制） |
| `max_planning_time` | double | 5.0 | 最大规划时长（s） |
| `movement_weight` | double | 1.0 | 步进代价的乘子，平衡移动距离与障碍惩罚 |

### DStarLitePathSearch（命名空间 `dstarlite_planner`）

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `connectivity` | string | "8" | 栅格连通性：4/8（2D），6/18/26（3D） |
| `heuristic` | string | "Diagonal" | 启发函数：Euclidean / Manhattan / Chebyshev / Diagonal |
| `goal_tolerance` | double | 0.5 | 到目标的接受距离（m） |
| `max_expansions` | int | 1000000 | 最大扩展节点数（-1 不限制） |
| `max_path_steps` | int | 10000 | 提取路径的最大步数（-1 不限制） |
| `max_planning_time` | double | 5.0 | 最大规划时长（s，0.0 不限制） |
| `movement_weight` | double | 1.0 | 移动代价权重，越大越偏好低地形代价 |
| `greedy_weight` | double | 1.0 | 启发式贪婪权重（1.0 最优，>1.0 更快但可能次优） |
| `terrain_weight` | double | 1.0 | 地形代价乘子（边代价 = 步长 × (movement_weight + terrain_weight × cost)） |

> D* Lite 的 `IWorldView` 需要额外实现 `rows()` / `cols()` / `layers()` 提供栅格维度，
> 供引擎预分配 g/rhs/obstacles 状态数组；A* / BFS / Detour 的地图适配器不受影响。

### DetourPlanner（命名空间 `detour_planner`）

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `scan_distance_m` | double | 5.0 | 沿参考轨迹向前扫描堵塞的最远距离（m） |
| `blockage_threshold` | double | 0.8 | 软堵塞：归一化代价（0-1）达到此值即触发绕行；`isWalkable()` 为 false 的路径点始终触发 |
| `reconnect_offset_m` | double | 1.0 | 绕行后重新挂接参考轨迹的距离（m），至少越过堵塞点这么多 |
| `obstacle_cost_weight` | double | 1.0 | 绕行搜索中地图代价的系数（0 = 忽略代价，越大越避障） |
| `max_planning_time` | double | 0.1 | 单次绕行搜索的最大耗时（s） |

> DetourPlanner 的 `IWorldView::getCost()` 要求返回归一化代价（如 TerrainAnalyzer 原始代价 / 254.0）。

## 示例

四个 planner 参数命名空间不同，可分别配置。下面是全部默认值：

```yaml
astar_planner:
  connectivity: "8"
  heuristic: "Euclidean"
  goal_tolerance: 0.5
  max_iterations: 1000000
  max_planning_time: 5.0
  movement_weight: 1.0
  greedy_weight: 1.001

bfs_planner:
  connectivity: "8"
  goal_tolerance: 0.5
  max_iterations: 1000000
  max_planning_time: 5.0
  movement_weight: 1.0

dstarlite_planner:
  connectivity: "8"
  heuristic: "Diagonal"
  goal_tolerance: 0.5
  max_expansions: 1000000
  max_path_steps: 10000
  max_planning_time: 5.0
  movement_weight: 1.0
  greedy_weight: 1.0
  terrain_weight: 1.0

detour_planner:
  scan_distance_m: 5.0
  blockage_threshold: 0.8
  reconnect_offset_m: 1.0
  obstacle_cost_weight: 1.0
  max_planning_time: 0.1
```
