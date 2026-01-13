渲染与重规划调度更新

此文档补充说明了对 `run.py` 中重规划（replanning）调度逻辑的调整，目的在于按用户要求在 T 帧周期内合理分配每帧需要执行的路径规划任务。

变更概述：
- 在 `run.py` 中用 `replan_frame` 取代原先的 `replan_cursor`，实现基于帧索引的重规划分配逻辑。

调度规则：
- 令 M 为无人机数量，T 为重规划周期帧数（`replan_T`）：
  - 当 M <= T 时：在每个 T 帧循环中，只有前 M 帧分别执行对编号 0..M-1 的无人机的路径规划；帧 M..T-1 不进行任何规划（使每架无人机在自己的专属帧被规划一次）。
  - 当 M > T 时：将 M 个无人机均匀分配到 T 帧中，每帧分配 ceil(M/T) 个无人机（即每帧执行 `per_frame = ceil(M/T)` 个），最后一帧可能分配到的无人机数量少于其他帧（当 M % T != 0）。

实现要点：
- `replan_frame = (replan_frame + 1) % T` 用于在每帧推进时轮动帧索引。
- 当 M <= T：如果 `frame_idx < M` 则 `indices = [frame_idx]`，否则 `indices = []`。
- 当 M > T：计算 `per_frame = ceil(M/T)`，并令 `start = frame_idx * per_frame; end = min(start+per_frame, M)`，`indices = list(range(start, end))`。

影响与理由：
- 该策略把“分摊规划开销”的粒度由基于游标的循环（旧逻辑）变为基于固定 T 帧周期的分配，更明确地支持“当无人机少于周期帧数时有空闲帧”的需求，同时保留对大量无人机时按帧均匀分配的能力。
- 保持对 `plan_paths_fn`（Python 或 C/ctypes 实现）的兼容：返回的子路径仍按 `indices` 映射回 `drones[ai].nav_points` 与 `planned_paths[ai]`。

可改进项（建议）：
- 若希望显式区分“nav_points 未设置”与“已设置为空”，可用 `hasattr(drone, 'nav_points')` 或把 `nav_points` 初始值设置为 `None`。
- 增加异常日志（在 catch 块中打印异常信息），便于排查规划失败原因。

修改位置：
- 文件：`run.py`（仓库根目录）
- 主要变量：`replan_frame`, `replan_T`（配置项 `rePlan_pathsT`）
总体渲染逻辑总结

- 渲染入口与初始化：
  - 程序在 run.py 中调用 renderer.init_display()（在 [scr/main.py](scr/main.py)）创建 screen, clock, font，并进入主循环。

- 每帧主循环（在 [run.py](run.py)）：
  - 事件处理（退出、按键、鼠标按钮等）。
  - 规划与物理更新：调用 plan_paths 与 compute_acceleration（默认 Python 实现在 [scr/algorithm.py](scr/algorithm.py)，可通过 config/set.json 将 ifpy=false 切换为 C/ctypes 实现）。
  - 计算雷达（radar）读数、更新无人机状态（位置/速度/加速度/历史）、处理到达目标并可能向环境中添加阻挡块。
  - 构建 state 字典（包括 starts, goals, obstacles, planned_paths, drones, sim_running, btn_rect, btn_font, show_history, show_radar, radar_data, history_color, history_width, SCALE, OFFSET, fps 等）。
  - 将 state 传入渲染器：renderer.draw_frame(screen, font, state)。

- 渲染实现（scr/main.py 中的 draw_frame）：
  - 清屏并使用 world_to_screen（基于 SCALE 和 OFFSET）做世界→屏幕坐标转换。
  - 绘制起点与终点（小圆）、绘制障碍（填充多边形与边框）、绘制规划路径（planned_paths 为折线）。
  - 绘制无人机：
    - 若启用 show_radar：使用 radar_data 在每个方向绘制雷达线与检测点（命中/未命中使用不同颜色）。
    - 若启用 show_history：用 drone.history 绘制轨迹线（history_color/history_width）。
    - 绘制无人机圆形、序号标签以及雷达数值文本。
  - 绘制控制按钮（根据 sim_running 显示 START 或 REPLACE）与右上角 FPS 文本。
  - 最后调用 pygame.display.flip() 刷新显示，主循环以 clock.tick(60) 限制帧率。

- 关键分工与可替换点：
  - run.py：负责仿真进程、感知（雷达）、规划调用、物理积分、构建 state（渲染无状态）。
  - scr/algorithm.py：提供 plan_paths（可见图 + Dijkstra + 采样）与 compute_acceleration（基于雷达的反应式控制），可替换为 C 实现（algorithmC）通过 ctypes 调用，接口不变。
  - scr/main.py：负责所有绘制细节与视觉样式，保持与仿真逻辑分离。
  - 地图编辑工具在 tool/mapMade.py，用于生成 set/*.json 地图与起终点数据。

- 快速定位（文件）：
  - 仿真与渲染循环：[run.py](run.py)
  - 绘制实现：[scr/main.py](scr/main.py)
  - 路径规划与控制：[scr/algorithm.py](scr/algorithm.py)
  - 地图编辑器：[tool/mapMade.py](tool/mapMade.py)
  - 算法说明：[wiki/algorithm_function.md](wiki/algorithm_function.md)

若需要我可：
- 将此文档改为中文/英文双语版本；
- 在仓库中添加更详细的代码注释或将关键绘制参数提取到配置文件；
- 运行演示并截图/录制小段视频用于验证（需在本机运行）。
