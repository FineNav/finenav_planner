# finenav_route_planner

栅格路径规划器集合，作为 FineNav-Engine 的 planner 插件运行，包含两个 planner：

- `AstarPathSearch` — 均匀栅格 A* 搜索
- `BfsPathSearch` — 无方向启发式的栅格搜索（A* + Heuristic::Zero，即 Dijkstra / 加权 BFS）

## 简介

两个 planner 都实现 FineNav 的 `Planner` 概念：声明 `IWorldView` 读取地图，`plan(ctx, world_view)` 返回轨迹。

- `AstarPathSearch` / `BfsPathSearch` 共用 `astar::IWorldView`（`isWalkable` / `getCost` / `getStepSize`），在均匀栅格上做搜索；BFS 等价于启发函数为零的 A*，均匀地图上即最少跳数路径
- 栅格步长由 `IWorldView::getStepSize()` 在规划时提供，不是参数

## 参数

两个 planner 参数命名空间不同，参数分别定义在 `astar.yaml`、`bfs.yaml`。

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

## 示例

两个 planner 参数命名空间不同，可分别配置。下面是全部默认值：

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
```
