import os
import json
import math
import copy
import pygame
from scr import algorithm
from scr import main as renderer

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SET_DIR = os.path.join(BASE_DIR, "set")
OBS_FILE = os.path.join(SET_DIR, "obstacles2.json")
POS_FILE = os.path.join(SET_DIR, "positions3.json")

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
                # flatten polygon vertices and counts
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
                # expected length = n_agents * steps * 2
                out_len = n_agents * steps * 2
                res = []
                if out_len > 0:
                    for i in range(n_agents):
                        path = []
                        for j in range(steps):
                            idx = (i*steps + j) * 2
                            x = ptr[idx]
                            y = ptr[idx+1]
                            path.append((x, y))
                        res.append(path)
                # free buffer
                plan_dll.free_buffer(ptr)
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
                        sim_running = True
                    else:
                        sim_running = False
                        for j, d in enumerate(drones):
                            d.pos[0] = float(starts[j][0])
                            d.pos[1] = float(starts[j][1])
                            d.vel = [0.0, 0.0]
                            d.acc = [0.0, 0.0]
                            d.arrived = False
                            d.block_added = False
                            d.history = [(float(starts[j][0]), float(starts[j][1]))]
                        obstacles[:] = copy.deepcopy(orig_obstacles)

        # per-frame computation
        current_starts = [[d.pos[0], d.pos[1]] for d in drones]
        planned_paths = plan_paths_fn(obstacles, current_starts, goals, steps=200)

        # compute radar data for visualization and control
        radar_data = []
        for d in drones:
            raw_readings, dirs = cast_radar(d.pos, obstacles, 8, d.radar)
            radar_data.append((raw_readings, dirs))

        for i, path in enumerate(planned_paths):
            if len(path) == 0:
                continue
            drone = drones[i]

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
                    dt = float(sim_rate) / 60.0
                    drone.update(dt)
                    drone.history.append((drone.pos[0], drone.pos[1]))
                    if len(drone.history) > history_max_len:
                        drone.history = drone.history[-history_max_len:]

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
        }

        # provide current FPS to renderer (clock.get_fps may reflect recent frames)
        state["fps"] = clock.get_fps()
        renderer.draw_frame(screen, font, state)
        clock.tick(60)

    pygame.quit()


if __name__ == "__main__":
    main()
