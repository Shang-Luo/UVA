import os
import json
import math
import copy
import pygame
from scr import algorithm
from scr import main as renderer
from scr.rrt_star import rrt_star

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

    # Use RRT* based planner (scr/rrt_star.py). Do not call algorithm.plan_paths.

    def _poly_to_bbox(poly):
        xs = [float(v[0]) for v in poly]
        ys = [float(v[1]) for v in poly]
        return {'type': 'rect', 'xmin': min(xs), 'ymin': min(ys), 'xmax': max(xs), 'ymax': max(ys)}

    def plan_paths_rrt_star(obstacles_py, starts_py, goals_py, indices_list=None, steps=200):
        # convert polygon obstacles to axis-aligned bounding boxes for rrt_star
        rrt_obs = [_poly_to_bbox(poly) for poly in obstacles_py]
        res = []
        # compute bounds from obstacles and starts/goals
        xs = []
        ys = []
        for poly in obstacles_py:
            for v in poly:
                xs.append(float(v[0])); ys.append(float(v[1]))
        for s in starts_py:
            xs.append(float(s[0])); ys.append(float(s[1]))
        for g in goals_py:
            xs.append(float(g[0])); ys.append(float(g[1]))
        if not xs:
            xmin, ymin, xmax, ymax = 0, 0, 1000, 1000
        else:
            pad = 50
            xmin, ymin, xmax, ymax = min(xs)-pad, min(ys)-pad, max(xs)+pad, max(ys)+pad

        # indices_list maps each start/goal to agent index to fetch its robot radius
        if indices_list is None:
            indices_list = list(range(len(starts_py)))

        for k, (s, g) in enumerate(zip(starts_py, goals_py)):
            agent_idx = indices_list[k] if k < len(indices_list) else None
            robot_r = 0.0
            try:
                if agent_idx is not None:
                    robot_r = float(drones[agent_idx].radius)
            except Exception:
                robot_r = 0.0
            # increase max_iter to improve chance of finding a path
            # set max iterations to large int (2^31-1) as requested
            max_iter_use = 1073741823
            path = rrt_star(tuple(s), tuple(g), rrt_obs, (xmin, ymin, xmax, ymax), max_iter=max_iter_use, step_size=8.0, search_radius=30.0, goal_sample_rate=0.05, robot_radius=robot_r, smooth=True, samples_per_segment=12)
            res.append(path if path is not None else [])
        return res

    plan_paths_fn = plan_paths_rrt_star
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
                    subset_paths = plan_paths_fn(obstacles, subset_starts, subset_goals, indices, steps=plan_step_max)
                else:
                    subset_paths = []
            except Exception:
                subset_paths = [[] for _ in indices]
            # assign back into drones' nav_points and planned_paths
            for j, ai in enumerate(indices):
                path = subset_paths[j] if j < len(subset_paths) else []
                drones[ai].nav_points = path
                planned_paths[ai] = path

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

            nearest_idx = 0
            best_d2 = float('inf')
            for idx, p in enumerate(path):
                dx = p[0] - drone.pos[0]
                dy = p[1] - drone.pos[1]
                d2 = dx*dx + dy*dy
                if d2 < best_d2:
                    best_d2 = d2
                    nearest_idx = idx

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
                lookahead = min(nearest_idx + 6, len(path)-1)
                target_pt = path[lookahead]
                target_vec = (target_pt[0] - drone.pos[0], target_pt[1] - drone.pos[1])

                if sim_running:
                    ax, ay = compute_acc_fn(readings, dirs, target_vec, tuple(drone.vel), params={"max_range": drone.radar})
                    # inter-drone repulsion
                    k_dd = 1000.0
                    eps = 1e-3
                    for j, od in enumerate(drones):
                        if j == i or od.arrived:
                            continue
                        dx = drone.pos[0] - od.pos[0]
                        dy = drone.pos[1] - od.pos[1]
                        dd = math.hypot(dx, dy)
                        if dd < 1e-6:
                            continue
                        str_rep = k_dd / (dd*dd + eps)
                        ax += (dx/dd) * str_rep
                        ay += (dy/dd) * str_rep

                    max_acc = 6.0
                    m = math.hypot(ax, ay)
                    if m > max_acc and m > 0:
                        s = max_acc / m
                        ax *= s
                        ay *= s

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
