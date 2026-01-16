import os
import json
import math
import copy
import pygame
from scr import algorithm
from scr import main as renderer

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SET_DIR = os.path.join(BASE_DIR, "set")
# 从配置文件读取数据文件名（可在 config/set.json 中配置 'obs_file' 和 'pos_file'），
# 若配置缺失则使用默认值
cfg_path = os.path.join(BASE_DIR, "config", "set.json")
_cfg = {}
if os.path.exists(cfg_path):
    try:
        with open(cfg_path, 'r', encoding='utf-8') as _cf:
            _cfg = json.load(_cf)
    except Exception:
        _cfg = {}

OBS_NAME = _cfg.get('obs_file', 'obstacles2.json')
POS_NAME = _cfg.get('pos_file', 'positions3.json')

OBS_FILE = os.path.join(SET_DIR, OBS_NAME)
POS_FILE = os.path.join(SET_DIR, POS_NAME)

# load data
with open(OBS_FILE, "r", encoding="utf-8") as f:
    obstacles = json.load(f)
with open(POS_FILE, "r", encoding="utf-8") as f:
    agents = json.load(f)

starts = [a["start"] for a in agents]
goals = [a["goal"] for a in agents]
radii = [a.get("radius", 6) for a in agents]
radar_ranges = [a.get("radar_range", 120) for a in agents]

# 交换 0 号与 2 号无人机的到达位置（若存在）
if len(goals) > 2:
    try:
        goals[0], goals[2] = goals[2], goals[0]
    except Exception:
        pass

class Drone:
    def __init__(self, start, radius=6, radar=120):
        self.pos = [float(start[0]), float(start[1])]
        self.vel = [0.0, 0.0]
        self.acc = [0.0, 0.0]
        self.radius = radius
        self.radar = radar
        self.arrived = False
        self.block_added = False
        self.history = [(self.pos[0], self.pos[1])]
        self.nav_points = []

    def update(self, dt=1.0):
        self.vel[0] += self.acc[0] * dt
        self.vel[1] += self.acc[1] * dt
        self.pos[0] += self.vel[0] * dt
        self.pos[1] += self.vel[1] * dt


# helpers for radar
def _cross(v, w):
    return v[0]*w[1] - v[1]*w[0]


def ray_segment_intersection(p, dir_unit, a, b, max_range):
    r = dir_unit
    s = (b[0]-a[0], b[1]-a[1])
    ap = (a[0]-p[0], a[1]-p[1])
    rxs = _cross(r, s)
    if abs(rxs) < 1e-8:
        return None
    t = _cross(ap, s) / rxs
    u = _cross(ap, r) / rxs
    if t >= 0 and 0.0 <= u <= 1.0:
        dist = t
        if dist <= max_range:
            ix = p[0] + r[0]*t
            iy = p[1] + r[1]*t
            return (dist, (ix, iy))
    return None


def cast_radar(pos, obstacles, n_dirs, max_range):
    readings = []
    dirs = []
    for k in range(n_dirs):
        ang = k * (2*math.pi / n_dirs)
        dir_unit = (math.cos(ang), math.sin(ang))
        dirs.append(dir_unit)
        min_d = max_range
        for poly in obstacles:
            for i in range(len(poly)):
                a = poly[i]
                b = poly[(i+1) % len(poly)]
                res = ray_segment_intersection(pos, dir_unit, a, b, max_range)
                if res is not None:
                    d, _pt = res
                    if d < min_d:
                        min_d = d
        readings.append(min_d)
    return readings, dirs

# 新增：路径平滑（Chaikin）与重采样，以及基于距离的前瞻目标选择
def smooth_path_chaikin(path, iterations=2):
    if not path or len(path) < 2:
        return path
    pts = [(float(p[0]), float(p[1])) for p in path]
    for _ in range(iterations):
        new = [pts[0]]
        for i in range(len(pts) - 1):
            p0 = pts[i]; p1 = pts[i+1]
            q = (0.75*p0[0] + 0.25*p1[0], 0.75*p0[1] + 0.25*p1[1])
            r = (0.25*p0[0] + 0.75*p1[0], 0.25*p0[1] + 0.75*p1[1])
            new.append(q); new.append(r)
        new.append(pts[-1])
        pts = new
    return pts

def resample_path(path, spacing=1.0):
    if not path or len(path) < 2:
        return path
    out = [tuple(path[0])]
    for i in range(len(path)-1):
        a = path[i]; b = path[i+1]
        dx = b[0] - a[0]; dy = b[1] - a[1]
        seg = math.hypot(dx, dy)
        if seg < 1e-6:
            continue
        n = int(math.floor(seg / spacing))
        for k in range(1, n+1):
            t = (k * spacing) / seg
            if t >= 1.0:
                continue
            out.append((a[0] + dx * t, a[1] + dy * t))
        out.append((b[0], b[1]))
    # 去重连续近似重复点
    res = [out[0]]
    for p in out[1:]:
        if math.hypot(p[0]-res[-1][0], p[1]-res[-1][1]) > 1e-6:
            res.append(p)
    return res

def smooth_and_resample(path):
    if not path:
        return path
    sp = smooth_path_chaikin(path, iterations=2)
    rp = resample_path(sp, spacing=1.0)
    return rp

def get_lookahead_point_along_path(path, pos, lookahead_dist=12.0):
    if not path:
        return None
    # 找到最近点索引并沿路径累加距离直到达到 lookahead_dist
    best_idx = 0
    best_d2 = float('inf')
    for idx, p in enumerate(path):
        d2 = (p[0]-pos[0])**2 + (p[1]-pos[1])**2
        if d2 < best_d2:
            best_d2 = d2; best_idx = idx
    dist_acc = 0.0
    cur = path[best_idx]
    # 如果已经接近末端，直接返回末端
    for j in range(best_idx, len(path)-1):
        a = path[j]; b = path[j+1]
        seg_len = math.hypot(b[0]-a[0], b[1]-a[1])
        if seg_len <= 1e-6:
            continue
        need = lookahead_dist - dist_acc
        if need <= 0:
            return cur
        if seg_len >= need:
            t = need / seg_len
            return (a[0] + (b[0]-a[0])*t, a[1] + (b[1]-a[1])*t)
        dist_acc += seg_len
        cur = b
    return path[-1]


def _inflate_poly(poly, dist):
    # 将多边形顶点相对于多边形重心按比例外扩（近似偏移）
    if not poly or dist == 0:
        return list(poly)
    cx, cy = _centroid(poly)
    out = []
    for x, y in poly:
        vx = x - cx; vy = y - cy
        n = math.hypot(vx, vy)
        if n < 1e-6:
            # 若顶点与中心重合，随意偏移一个方向
            nx, ny = cx + dist, cy
        else:
            nx = x + (vx / n) * dist
            ny = y + (vy / n) * dist
        out.append((nx, ny))
    return out


def inflate_obstacles(obstacles, inflation):
    if inflation <= 0:
        return [list(poly) for poly in obstacles]
    result = []
    for poly in obstacles:
        try:
            result.append(_inflate_poly(poly, inflation))
        except Exception:
            result.append(list(poly))
    return result


def simplify_path(path, obstacles):
    # 通过尝试连接更远的顶点来简化路径，前提是直线段不穿过任何障碍
    if not path or len(path) <= 2:
        return path
    simplified = []
    i = 0
    n = len(path)
    while i < n:
        simplified.append(path[i])
        # find farthest j we can connect to from i
        j = n - 1
        found = False
        while j > i:
            a = path[i]
            b = path[j]
            try:
                if not algorithm._segment_crosses_poly(a, b, obstacles):
                    # reachable directly
                    i = j
                    found = True
                    break
            except Exception:
                # 保守策略：若检测失败则降级到连接到下一个点
                pass
            j -= 1
        if not found:
            i += 1
    # always include final point
    if simplified[-1] != path[-1]:
        simplified.append(path[-1])
    # 去重相邻重复点
    out = [simplified[0]]
    for p in simplified[1:]:
        if math.hypot(p[0]-out[-1][0], p[1]-out[-1][1]) > 1e-6:
            out.append(p)
    return out


def load_config():
    cfg_file = os.path.join(BASE_DIR, "config", "set.json")
    cfg = {}
    if os.path.exists(cfg_file):
        try:
            with open(cfg_file, "r", encoding="utf-8") as cf:
                cfg = json.load(cf)
        except Exception:
            cfg = {}
    return cfg


def main():
    cfg = load_config()
    sim_rate = cfg.get("simulation_rate", 1)
    use_py = cfg.get("ifpy", True)
    show_history = cfg.get("show_history", True)
    show_radar = cfg.get("show_radar", True)
    hist_cfg = cfg.get("history", {})
    history_color = tuple(hist_cfg.get("color", [50,120,255]))
    history_width = int(hist_cfg.get("width", 2))
    history_max_len = int(hist_cfg.get("max_length", 1000))

    drones = [Drone(starts[i], radii[i], radar_ranges[i]) for i in range(len(starts))]
    orig_obstacles = copy.deepcopy(obstacles)

    # planning configuration
    replan_T = int(cfg.get("rePlan_pathsT", 10))
    plan_step_max = int(cfg.get("plan_step_max", 20))
    # planning bookkeeping
    planned_paths = [[] for _ in drones]
    # replan_frame records the current frame index inside a T-frame replan cycle (0..T-1)
    replan_frame = 0

    # If `ifpy` is false, prepare ctypes wrappers to call compiled C DLLs
    plan_paths_fn = algorithm.plan_paths
    compute_acc_fn = algorithm.compute_acceleration
    if not use_py:
        try:
            import ctypes
            from ctypes import c_double, c_int, POINTER

            base = BASE_DIR
            plan_dll_path = os.path.join(base, "algorithmC", "plan_paths.dll")
            acc_dll_path = os.path.join(base, "algorithmC", "acceleration.dll")

            plan_dll = ctypes.CDLL(plan_dll_path)
            acc_dll = ctypes.CDLL(acc_dll_path)

            # plan_paths_c signature:
            # double* plan_paths_c(const double* verts_flat, const int* poly_counts, int n_polys, int total_vertices, const double* starts, const double* goals, int n_agents, int steps)
            plan_dll.plan_paths_c.restype = POINTER(c_double)
            plan_dll.plan_paths_c.argtypes = [POINTER(c_double), POINTER(c_int), c_int, c_int, POINTER(c_double), POINTER(c_double), c_int, c_int]
            # free helper
            plan_dll.free_buffer.restype = None
            plan_dll.free_buffer.argtypes = [POINTER(c_double)]

            # compute_acceleration_c signature:
            # void compute_acceleration_c(const double* radar_readings, const double* directions, int N, const double* target_vec, const double* vel, double* out_axay)
            acc_dll.compute_acceleration_c.restype = None
            acc_dll.compute_acceleration_c.argtypes = [POINTER(c_double), POINTER(c_double), c_int, POINTER(c_double), POINTER(c_double), POINTER(c_double)]

            def plan_paths_c_wrapper(obstacles_py, starts_py, goals_py, steps=200):
                # Pack obstacles verts
                poly_counts = []
                verts = []
                for poly in obstacles_py:
                    poly_counts.append(len(poly))
                    for v in poly:
                        verts.extend([float(v[0]), float(v[1])])
                total_vertices = sum(poly_counts)
                n_polys = len(poly_counts)

                verts_arr = (c_double * (len(verts)))(*verts) if verts else (c_double * 0)()
                counts_arr = (c_int * n_polys)(*poly_counts) if n_polys>0 else (c_int * 0)()

                starts_flat = []
                for s in starts_py:
                    starts_flat.extend([float(s[0]), float(s[1])])
                goals_flat = []
                for g in goals_py:
                    goals_flat.extend([float(g[0]), float(g[1])])

                starts_arr = (c_double * (len(starts_flat)))(*starts_flat) if starts_flat else (c_double * 0)()
                goals_arr = (c_double * (len(goals_flat)))(*goals_flat) if goals_flat else (c_double * 0)()

                n_agents = len(starts_py)
                ptr = plan_dll.plan_paths_c(verts_arr, counts_arr, c_int(n_polys), c_int(total_vertices), starts_arr, goals_arr, c_int(n_agents), c_int(steps))
                res = []
                if not ptr:
                    return [[] for _ in range(n_agents)]
                # New C format: [n_agents(double), counts[0..n_agents-1] (double), coords...]
                try:
                    returned_n = int(ptr[0])
                except Exception:
                    plan_dll.free_buffer(ptr)
                    return [[] for _ in range(n_agents)]
                counts = []
                for i in range(returned_n):
                    counts.append(int(ptr[1 + i]))
                offset = 1 + returned_n
                coord_idx = offset
                for i in range(returned_n):
                    cnt = counts[i]
                    path = []
                    for j in range(cnt):
                        x = ptr[coord_idx + (j*2) + 0]
                        y = ptr[coord_idx + (j*2) + 1]
                        path.append((x, y))
                    coord_idx += cnt * 2
                    res.append(path)
                plan_dll.free_buffer(ptr)
                # If returned fewer agents than requested, pad
                if len(res) < n_agents:
                    res.extend([[] for _ in range(n_agents - len(res))])
                return res

            def compute_acc_c_wrapper(readings, dirs, target_vec, vel, params=None):
                N = len(readings)
                # flatten readings and directions
                rd_arr = (c_double * N)(*map(float, readings)) if N>0 else (c_double * 0)()
                dir_flat = []
                for d in dirs:
                    dir_flat.extend([float(d[0]), float(d[1])])
                dir_arr = (c_double * (len(dir_flat)))(*dir_flat) if dir_flat else (c_double * 0)()
                tgt_arr = (c_double * 2)(float(target_vec[0]), float(target_vec[1]))
                vel_arr = (c_double * 2)(float(vel[0]), float(vel[1]))
                out_arr = (c_double * 2)()
                acc_dll.compute_acceleration_c(rd_arr, dir_arr, c_int(N), tgt_arr, vel_arr, out_arr)
                return (float(out_arr[0]), float(out_arr[1]))

            plan_paths_fn = plan_paths_c_wrapper
            compute_acc_fn = compute_acc_c_wrapper
        except Exception:
            # fallback to python implementations on any failure
            plan_paths_fn = algorithm.plan_paths
            compute_acc_fn = algorithm.compute_acceleration

    screen, clock, font = renderer.init_display()
    btn_rect = pygame.Rect(10, 10, 90, 30)
    btn_font = pygame.font.SysFont(None, 20)

    SCALE = 1.0
    OFFSET = (0, 0)

    sim_running = False
    sim_time = 0.0
    timer_running = False

    running = True
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            if ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                running = False
            if ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                if btn_rect.collidepoint(ev.pos):
                    if not sim_running:
                        # start simulation: reset timer and start counting
                        sim_running = True
                        sim_time = 0.0
                        timer_running = True
                    else:
                        # stop / reset simulation: stop timer and reset drones/obstacles
                        sim_running = False
                        timer_running = False
                        for j, d in enumerate(drones):
                            d.pos[0] = float(starts[j][0])
                            d.pos[1] = float(starts[j][1])
                            d.vel = [0.0, 0.0]
                            d.acc = [0.0, 0.0]
                            d.arrived = False
                            d.block_added = False
                            d.history = [(float(starts[j][0]), float(starts[j][1]))]
                        obstacles[:] = copy.deepcopy(orig_obstacles)

        # per-frame computation: incremental replanning to limit work per frame
        # frame timestep used for simulation updates (seconds)
        frame_dt = float(sim_rate) / 60.0
        current_starts = [[d.pos[0], d.pos[1]] for d in drones]
        M = len(drones)
        T = max(1, replan_T)
        # select indices to replan this frame
        indices = []
        if M == 0:
            indices = []
        else:
            if M <= T:
                # When there are fewer drones than frames in a cycle,
                # only frames 0..M-1 perform planning for drones 0..M-1 respectively;
                # frames M..T-1 do nothing.
                frame_idx = replan_frame % T
                if frame_idx < M:
                    indices = [frame_idx]
                else:
                    indices = []
            else:
                # When M > T, distribute agents across the T frames roughly evenly.
                per_frame = int(math.ceil(float(M) / float(T)))
                frame_idx = replan_frame % T
                start = frame_idx * per_frame
                end = min(start + per_frame, M)
                if start < end:
                    indices = list(range(start, end))

        if len(indices) > 0:
            # 过滤掉已到达的无人机，避免为已到达的无人机重新规划或拉取导航点
            indices = [i for i in indices if not drones[i].arrived]
            if len(indices) == 0:
                subset_paths = []
            else:
                subset_starts = [current_starts[i] for i in indices]
                subset_goals = [goals[i] for i in indices]
            try:
                # 如果 indices 为空，上面已置空 subset_paths
                if len(indices) > 0:
                    subset_paths = []
                    # 为每个无人机单独规划：使用按该无人机半径膨胀的障碍，确保无人机表面及触角有足够边距
                    safety_margin = 4.0
                    for ai, s, g in zip(indices, subset_starts, subset_goals):
                        try:
                            # 对 2 号无人机使用更大的安全边距，保证触角不会接触障碍
                            sm = safety_margin
                            if ai == 2:
                                sm = max(sm, 10.0)
                            inf = (radii[ai] if ai < len(radii) else max(radii) if radii else 0) + sm
                        except Exception:
                            inf = safety_margin
                        inf_obs = inflate_obstacles(obstacles, inf)
                        p_res = plan_paths_fn(inf_obs, [s], [g], steps=plan_step_max)
                        # plan_paths_fn returns list-of-paths; extract first
                        path0 = p_res[0] if p_res and len(p_res) > 0 else []
                        # 在同样的膨胀障碍上做简化（确保保留边距），然后平滑重采样
                        try:
                            if path0:
                                simp = simplify_path(path0, inf_obs)
                                smooth = smooth_and_resample(simp)
                                subset_paths.append(smooth)
                            else:
                                subset_paths.append([])
                        except Exception:
                            subset_paths.append(path0 if path0 else [])
                else:
                    subset_paths = []
            except Exception:
                subset_paths = [[] for _ in indices]
            # assign back into drones' nav_points and planned_paths
            for j, ai in enumerate(indices):
                path = subset_paths[j] if j < len(subset_paths) else []
                # 平滑并重采样以得到连续曲线（减少折点与中途停顿）
                if path:
                    try:
                        smooth = smooth_and_resample(path)
                    except Exception:
                        smooth = path
                    drones[ai].nav_points = smooth
                    planned_paths[ai] = smooth
                else:
                    drones[ai].nav_points = []
                    planned_paths[ai] = []

        # advance frame index for next frame's replan selection
        replan_frame = (replan_frame + 1) % T

        # compute radar data for visualization and control
        radar_data = []
        for d in drones:
            raw_readings, dirs = cast_radar(d.pos, obstacles, 8, d.radar)
            radar_data.append((raw_readings, dirs))

        for i, drone in enumerate(drones):
            path = drone.nav_points if getattr(drone, 'nav_points', None) else planned_paths[i]
            if not path:
                continue

            # 使用基于距离的 lookahead 而非固定索引，令航迹更平滑
            lookahead_distance = max(12.0, drone.radius * 3.0)
            target_pt = get_lookahead_point_along_path(path, drone.pos, lookahead_distance)
            if target_pt is None:
                continue

            # reached goal check
            spos = (int(drone.pos[0]*SCALE + OFFSET[0]), int(drone.pos[1]*SCALE + OFFSET[1]))
            goal_world = goals[i]
            gpos = (int(goal_world[0]*SCALE + OFFSET[0]), int(goal_world[1]*SCALE + OFFSET[1]))
            if (not drone.arrived) and (math.hypot(spos[0]-gpos[0], spos[1]-gpos[1]) <= 5):
                drone.pos[0] = float(goal_world[0])
                drone.pos[1] = float(goal_world[1])
                drone.vel = [0.0, 0.0]
                drone.acc = [0.0, 0.0]
                drone.arrived = True
                # 清空该无人机的导航点与已计划路径，避免渲染黄色路径残留
                try:
                    drone.nav_points = []
                    planned_paths[i] = []
                except Exception:
                    pass
                if not drone.block_added:
                    h = drone.radius
                    cx, cy = goal_world[0], goal_world[1]
                    sq = [[cx-h, cy-h], [cx+h, cy-h], [cx+h, cy+h], [cx-h, cy+h]]
                    obstacles.append(sq)
                    drone.block_added = True
                    drone.history.append((drone.pos[0], drone.pos[1]))
                    if len(drone.history) > history_max_len:
                        drone.history = drone.history[-history_max_len:]

            if not drone.arrived:
                raw_readings, dirs = radar_data[i]
                readings = [max(r - drone.radius, 0.01) for r in raw_readings]
                # target_pt 已由 lookahead 距离选出
                # 若距离太近则退化为路径末端
                # (target_pt 已在上面计算)

                target_vec = (target_pt[0] - drone.pos[0], target_pt[1] - drone.pos[1])

                # 预测性障碍避让：基于速度/朝向计算前瞻点，若前瞻线段与任一障碍相交，则调整 target_vec
                try:
                    # 选择前瞻方向：优先使用速度方向，否则使用目标方向
                    speed = math.hypot(drone.vel[0], drone.vel[1])
                    if speed > 0.5:
                        dir_vel = (drone.vel[0]/speed, drone.vel[1]/speed)
                    else:
                        tvx, tvy = target_vec
                        tnorm = math.hypot(tvx, tvy) or 1.0
                        dir_vel = (tvx/tnorm, tvy/tnorm)

                    lookahead_time_obs = 1.5
                    # 对 2 号无人机增加前瞻时间以提前规避
                    if i == 2:
                        lookahead_time_obs = 2.5
                    lookahead_dist_obs = max(20.0, speed * lookahead_time_obs + drone.radius * 2.0)
                    look_pt = (drone.pos[0] + dir_vel[0] * lookahead_dist_obs, drone.pos[1] + dir_vel[1] * lookahead_dist_obs)

                    # 使用算法模块的多边形相交检测函数进行预测
                    if algorithm._segment_crosses_poly((drone.pos[0], drone.pos[1]), look_pt, obstacles):
                        # 找到第一个相交的障碍，朝该多边形重心反方向偏航
                        for poly in obstacles:
                            try:
                                if algorithm._segment_crosses_poly((drone.pos[0], drone.pos[1]), look_pt, poly):
                                    c = algorithm._centroid(poly)
                                    away = (drone.pos[0] - c[0], drone.pos[1] - c[1])
                                    da = math.hypot(away[0], away[1]) or 1.0
                                    steer = (away[0]/da, away[1]/da)
                                    # 将目标向量朝避让方向混合一些分量，强度与前瞻距离成比例
                                    blend = 1.8
                                    # 2号无人机给予更强的偏航混合，确保提前躲避
                                    if i == 2:
                                        blend = 3.2
                                    target_vec = (target_vec[0] + steer[0] * blend * (lookahead_dist_obs * 0.02),
                                                  target_vec[1] + steer[1] * blend * (lookahead_dist_obs * 0.02))
                                    break
                            except Exception:
                                continue
                except Exception:
                    # 若预测逻辑出错，不影响正常控制流程
                    pass

                if sim_running:
                    ax, ay = compute_acc_fn(readings, dirs, target_vec, tuple(drone.vel), params={"max_range": drone.radar})
                    # inter-drone 近距离斥力 + 预测式避碰（预测短时间位置）
                    k_dd = 800.0
                    eps = 1e-3
                    dt_pred = 0.5
                    for j, od in enumerate(drones):
                        if j == i or od.arrived:
                            continue
                        # 当前位置斥力
                        dx = drone.pos[0] - od.pos[0]; dy = drone.pos[1] - od.pos[1]
                        dd = math.hypot(dx, dy)
                        if dd > 1e-6:
                            str_rep = k_dd / (dd*dd + eps)
                            ax += (dx/dd) * str_rep
                            ay += (dy/dd) * str_rep
                        # 预测式避碰（基于短时间速度预测）
                        pred_self_x = drone.pos[0] + drone.vel[0] * dt_pred
                        pred_self_y = drone.pos[1] + drone.vel[1] * dt_pred
                        pred_od_x = od.pos[0] + od.vel[0] * dt_pred
                        pred_od_y = od.pos[1] + od.vel[1] * dt_pred
                        pdx = pred_self_x - pred_od_x; pdy = pred_self_y - pred_od_y
                        pdd = math.hypot(pdx, pdy)
                        safe = (drone.radius + od.radius) + 8.0
                        if pdd < safe and pdd > 1e-6:
                            strength = 200.0 * (safe - pdd) / safe
                            ax += (pdx / pdd) * strength
                            ay += (pdy / pdd) * strength

                    max_acc = 6.0
                    m = math.hypot(ax, ay)
                    if m > max_acc and m > 0:
                        s = max_acc / m
                        ax *= s
                        ay *= s

                    # 防止中途停顿：若速度与加速度都接近零，添加小的推进项朝向目标
                    speed = math.hypot(drone.vel[0], drone.vel[1])
                    acc_mag = math.hypot(ax, ay)
                    if speed < 0.6 and acc_mag < 0.5:
                        tx, ty = target_vec
                        td = math.hypot(tx, ty)
                        if td > 1e-6:
                            small_push = 1.2
                            ax += (tx/td) * small_push
                            ay += (ty/td) * small_push

                    drone.acc[0] = ax
                    drone.acc[1] = ay
                    dt = frame_dt
                    drone.update(dt)
                    drone.history.append((drone.pos[0], drone.pos[1]))
                    if len(drone.history) > history_max_len:
                        drone.history = drone.history[-history_max_len:]

        # 如果计时正在运行且并非所有无人机均已到位，则累加仿真时间
        all_arrived = all([d.arrived for d in drones]) if drones else True
        if timer_running and sim_running and (not all_arrived):
            sim_time += frame_dt

        # prepare renderer state
        state = {
            "starts": starts,
            "goals": goals,
            "obstacles": obstacles,
            "planned_paths": planned_paths,
            "drones": drones,
            "sim_running": sim_running,
            "btn_rect": btn_rect,
            "btn_font": btn_font,
            "show_history": show_history,
            "show_radar": show_radar,
            "radar_data": radar_data,
            "history_color": history_color,
            "history_width": history_width,
            "SCALE": SCALE,
            "OFFSET": OFFSET,
            "sim_time": sim_time,
        }

        # provide current FPS to renderer (clock.get_fps may reflect recent frames)
        state["fps"] = clock.get_fps()
        renderer.draw_frame(screen, font, state)
        clock.tick(60)

    pygame.quit()


if __name__ == "__main__":
    main()
