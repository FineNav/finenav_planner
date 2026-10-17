# finenav_planner

[FineNav-Engine](https://github.com/FineNav/FineNav-Engine) 的规划器插件集合，提供栅格路径搜索、局部绕障、朝向对齐和 MPPI 轨迹优化。

## 规划器

| 包 | 规划器 | 注册名 / 参数命名空间 | 用途 |
|---|---|---|---|
| `finenav_route_planner` | `AstarPathSearch` | `astar_planner` | 均匀栅格 A* 路径搜索 |
| `finenav_route_planner` | `BfsPathSearch` | `bfs_planner` | 零启发式 Dijkstra 搜索，按累计代价扩展 |
| `finenav_route_planner` | `DStarLitePathSearch` | `dstarlite_planner` | 复用搜索状态，同步障碍变化后增量重规划 |
| `finenav_route_planner` | `DetourPlanner` | `detour_planner` | 在参考路径上检测堵塞并生成二维绕行连接段 |
| `finenav_rotate_shim` | `RotateShim` | `finenav_rotate_shim` | 跟踪路径前原地旋转，对齐前方采样点方向 |
| `finenav_mppi_SE2` | `MPPISE2` | `finenav_mppi_SE2` | 在 SE(2) 空间中优化局部轨迹和控制序列 |

按任务选择 A*、Dijkstra 或 D* Lite 生成路径；Detour 用于调整参考路径，RotateShim 用于跟踪前的朝向对齐，MPPI 负责局部轨迹优化。调用顺序由 Engine 的任务和策略配置决定。

## 使用说明

- [栅格规划与局部绕障](finenav_route_planner/README.md)：地图接口、参数、测试和算法评估。
- [MPPI](finenav_mppi_SE2/README.md)：运动模型、critic 配置和 WorldView 要求。
- [RotateShim](finenav_rotate_heading_aligner/README.md)：旋转参数及规划结果约定。

`finenav_rotate_heading_aligner/` 目录中的 ROS 包名为 `finenav_rotate_shim`。

## 构建

将本仓库与 FineNav-Engine 的依赖包放在同一个 colcon 工作区，并加载 ROS 2 环境。构建需要支持 C++20 的编译器、CMake 3.22 或更新版本，以及 Eigen3、OpenCV、OpenMP 和 `generate_parameter_library`。启用测试时还需 `ament_cmake_gtest` 和 Google Benchmark。

在工作区根目录执行：

```bash
colcon build --symlink-install --packages-up-to \
  finenav_route_planner finenav_mppi_SE2 finenav_rotate_shim
source install/setup.bash
```

各包通过 FineNav 的规划器注册机制接入 Engine，配置时使用上表中的注册名和对应参数命名空间。地图接口由使用方适配，具体要求见各包说明。

## License

FineNav-authored code and modifications are licensed under the
[Mozilla Public License 2.0 (MPL-2.0)](LICENSE).

Third-party code retains its original copyright and license notices:

- MPPI: see [NOTICE](finenav_mppi_SE2/NOTICE),
  [MIT](finenav_mppi_SE2/LICENSES/MIT.txt), and
  [Apache-2.0](finenav_mppi_SE2/LICENSES/Apache-2.0.txt).
- RotateShim: see [NOTICE](finenav_rotate_heading_aligner/NOTICE) and
  [Apache-2.0](finenav_rotate_heading_aligner/LICENSES/Apache-2.0.txt).
