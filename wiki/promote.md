**RRT* 实现说明（基于 scr/rrt_star.py）**

本文档介绍 `scr/rrt_star.py` 中实现的关键模块、设计思路、主要函数与使用方法，便于理解、复现与改进。

**一、总体概述**
- 目标：实现 RRT* 树搜索以在有多边形障碍的平面内寻找从起点到目标的可行路径；支持考虑机器人半径的碰撞检测、用贝塞尔曲线对结果路径进行平滑。
- 关键文件：
  - `scr/rrt_star.py` — RRT* 的 Python 实现（`Node`、`RRTStar`、碰撞检测与贝塞尔平滑函数）。

**二、核心数据结构**
- `Node`：保存 `x, y`，父节点 `parent` 与当前路径代价 `cost`。
- `RRTStar`：主要成员
  - `start`, `goal`：`Node` 对象
  - `rand_area`：采样范围 `(min_x, max_x, min_y, max_y)`
  **RRT* 实现说明（贴合 scr/rrt_star.py）**

  本文档以仓库中 `scr/rrt_star.py` 的真实实现为蓝本，按一致的模板与风格说明关键点、调用方式与可扩展方向，便于开发者直接对接与调优。

  **一、概述**
  - 目标：在平面带障碍环境中使用 RRT* 寻路，支持机器人半径感知的碰撞检测，并可对结果路径进行贝塞尔平滑。
  - 约定：
    - 起点/终点与路径点均为 `(x, y)` 二元元组；
    - 障碍支持圆（tuple (x,y,r) 或 dict）和轴对齐矩形（`{'type':'rect','xmin','ymin','xmax','ymax'}`）；
    - `robot_radius` 会膨胀障碍（等价于机器人为点）。

  **二、文件与主要函数（对应代码）**
  - `rrt_star(start, goal, obstacles, bounds, max_iter=500, step_size=1.0, search_radius=5.0, goal_sample_rate=0.05, robot_radius=0.0, smooth=False, samples_per_segment=8)`：主接口，返回点列表或 `None`。
  - 工具函数：
    - `_dist(a,b)`：欧氏距离；
    - `nearest(nodes, point)`：返回最近节点索引；
    - `steer(from_pt, to_pt, step_size)`：限制步长的点扩展；
    - `near(nodes, point, radius)`：返回半径内节点索引列表；
    - `get_path(nodes, goal_idx)`：回溯生成路径。
  - 碰撞相关：
    - `_seg_point_distance(a,b,p)` 与 `_seg_intersect(...)`：线段-点距离与相交检测；
    - `collision_free(a,b,obstacles,robot_radius=0.0)`：将圆形障碍膨胀、矩形障碍扩展 `robot_radius` 后判断线段是否与任何障碍冲突。
  - RRT* 子模块：
    - `choose_parent(nodes,new_pt,near_idxs,obstacles,robot_radius=0.0)`：在邻域节点中选择最优父节点（考虑碰撞与代价）；
    - `rewire(nodes,new_idx,near_idxs,obstacles,robot_radius=0.0)`：尝试用新节点改善邻居代价；
    - `smooth_path_bezier(path,samples_per_segment=8)`：Catmull‑Rom→Bezier 转换并采样得到平滑路径。

  **三、运行与集成要点**
  - 在 `run.py` 中已提供 `plan_paths_rrt_star` 封装：
    - 将多边形障碍转换为轴对齐包围矩形传入 `rrt_star`；
    - 根据 agent 索引读取 `Drone.radius` 作为 `robot_radius`；
    - 在规划时可以设置 `smooth=True` 与合适的 `samples_per_segment`。

  示例调用：
  ```py
  from scr.rrt_star import rrt_star
  path = rrt_star(start, goal, obstacles, bounds, max_iter=10000, step_size=8.0, search_radius=30.0, robot_radius=13.0, smooth=True, samples_per_segment=12)
  ```

  **四、参数建议**
  - `step_size`：依据场景尺度选择（示例使用 8~10）；
  - `search_radius`：控制 rewiring 邻域，示例使用 30；
  - `max_iter`：交互式仿真避免过大阻塞主线程，长离线规划可增大；
  - `samples_per_segment`：平滑分辨率，越大越平滑但点越多。

  **五、扩展与优化方向（贴合代码）**
  - 将 `nearest` / `near` 替换为 KD-Tree 或 spatial index；
  - 使用 Minkowski 膨胀替代运行时膨胀以获得更准确障碍边界；
  - 在平滑后做碰撞再次检测并局部优化（以避免因平滑导致的轻微穿越）；
  - 将 `smooth_path_bezier` 替换为基于优化的时间参数化以满足动力学约束。

  **六、文件位置（快速参考）**
  - `scr/rrt_star.py` — 实现文件（主接口 `rrt_star`、碰撞、平滑函数）；

  如需我把该说明合并到主 README 或为 `run.py` 提供开关配置（例如 `SMOOTH_PATH=True`），我可以继续修改。
