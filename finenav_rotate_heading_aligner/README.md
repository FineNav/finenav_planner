# finenav_rotate_shim

原地旋转规划器，作为 FineNav-Engine 的 planner 插件运行。

## 简介

在开始跟踪参考路径前，先原地旋转，使机器人朝向对准参考路径前方采样点的方向。对齐后 `plan()` 返回 `nullopt`，由上层 TrackingPolicy 结束本阶段并切换到后续 planner（如 MPPI）。

- 无地图依赖：不声明 `IWorldView`，`plan(ctx)` 只读机器人状态和参考路径
- 在参考路径上采样前方 `forward_sampling_distance` 处的点作为旋转目标方向
- 匀速旋转，直到航向误差小于 `angular_dist_threshold` 视为对齐
- 注册为 planner 插件 `finenav::RotateShim`，参数命名空间 `finenav_rotate_shim`

## 参数

参数定义在 `rotate_shim.yaml`，命名空间 `finenav_rotate_shim`。

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `forward_sampling_distance` | double | 0.5 | 参考路径上前方采样距离（m），决定旋转目标方向 |
| `angular_speed` | double | 0.8 | 匀速旋转的角速度（rad/s） |
| `angular_dist_threshold` | double | 0.1 | 航向误差小于该值时视为对齐（rad） |

## 示例

参数通过 ROS 2 参数文件加载，以 `finenav_rotate_shim` 为前缀声明。下面是全部默认值：

```yaml
finenav_rotate_shim:
  forward_sampling_distance: 0.5
  angular_speed: 0.8
  angular_dist_threshold: 0.1
```
