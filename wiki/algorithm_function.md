# Algorithm functions

此文档说明 `plan_paths` 与 `compute_acceleration` 两个函数的输入、输出与作用。

**Python 接口**

- `plan_paths(obstacles, starts, goals, steps=200)`
  - 输入：
    - `obstacles`: 列表，每个元素为多边形顶点列表（[[x,y], ...]）
    - `starts`: 列表，N 个起点，每个为 `[x,y]`
    - `goals`: 列表，N 个目标点，每个为 `[x,y]`
    - `steps`（可选）：每条路径采样点数量（整数）
  - 输出：
    - 返回长度为 N 的列表，每个元素为路径点序列（每个点为 `(x,y)`），路径长度约为 `steps`。
  - 作用：
    - 使用可见图（visibility graph）对每个 start->goal 规划碰撞避让路径。
    - 若直线可通行则直接线性插值；否则构建顶点图并在其上用 Dijkstra 最短路径，再按长度采样成 `steps` 个点。

- `compute_acceleration(radar_readings, directions, target_vec, vel, params=None)`
  - 输入：
    - `radar_readings`: 长度为 N 的距离列表（每个方向的探测距离）
    - `directions`: 长度为 N 的单位向量列表，对应每个雷达方向，格式 `(dx,dy)`
    - `target_vec`: 目标向量 `(dx,dy)`，指向当前导航点
    - `vel`: 当前速度 `(vx,vy)`
    - `params`（可选）：字典，可包含 `k_goal`, `avoid_gain`, `max_acc`, `max_range` 等调参项
  - 输出：
    - 返回二元组 `(ax, ay)`，表示合成加速度。
  - 作用：
    - 生成一个简单的反应式控制器：朝目标方向产生吸引力，同时基于雷达读数对障碍产生反作用（近处强），并对速度做阻尼，最后限制最大加速度。

**C 接口（用于 ctypes 调用）示例**

- plan_paths_c
  - 签名：
    - `double* plan_paths_c(const double* verts_flat, const int* poly_counts, int n_polys, int total_vertices, const double* starts, const double* goals, int n_agents, int steps)`
  - 参数说明：
    - `verts_flat`: 所有多边形顶点按序展平为 `[x0,y0,x1,y1,...]`（实现可选择忽略）
    - `poly_counts`: 每个多边形顶点数量的数组
    - `n_polys`: 多边形数量
    - `total_vertices`: 所有顶点总数
    - `starts`: `n_agents*2` 长度的数组，`[sx0,sy0,sx1,sy1,...]`
    - `goals`: 同上
    - `n_agents`: 无人机数量
    - `steps`: 每条路径采样点个数
  - 返回值：
    - 返回分配的 `double*`，长度为 `n_agents * steps * 2`，顺序为每个 agent 的 `steps` 个 `(x,y)` 对。
    - 调用方必须使用库提供的 `free_buffer(double*)` 释放。

- compute_acceleration_c
  - 签名：
    - `void compute_acceleration_c(const double* radar_readings, const double* directions, int N, const double* target_vec, const double* vel, double* out_axay)`
  - 参数说明：
    - `radar_readings`: 长度 N 的距离数组
    - `directions`: 长度 `2*N` 的数组，每个方向为 `(dx,dy)` 连续排列
    - `N`: 方向数量
    - `target_vec`: 长度 2 的数组 `(dx,dy)`
    - `vel`: 长度 2 的数组 `(vx,vy)`
    - `out_axay`: 长度 2 的输出数组，库将写入 `(ax, ay)`

生成的 C 文件位于：`algorithmC/plan_paths/plan_paths.c` 和 `algorithmC/acceleration/acceleration.c`。

备注：
- Python 侧 `run.py` 已加入 `ifpy` 配置开关（`config/set.json`）：默认 `true`。
- 当 `ifpy=false` 时，`run.py` 会尝试从 `algorithmC/plan_paths.dll` 和 `algorithmC/acceleration.dll` 加载 `plan_paths_c` 与 `compute_acceleration_c`，并使用 ctypes 进行数据传递。若加载或调用失败，会回退到 Python 实现。
