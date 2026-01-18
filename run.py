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

def is_point_in_polygon(point, polygon):
    """
    判断点是否在多边形内部（射线法）
    """
    x, y = point
    n = len(polygon)
    inside = False
    
    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    
    return inside

def filter_triangles_by_obstacles(triangles, obstacles):
    """
    过滤掉重心位于障碍物内部的三角形
    """
    filtered_tris = []
    
    for tri in triangles:
        # 1. 计算三角形重心
        a, b, c = tri
        centroid_x = (a[0] + b[0] + c[0]) / 3.0
        centroid_y = (a[1] + b[1] + c[1]) / 3.0
        centroid = (centroid_x, centroid_y)
        
        # 2. 判断重心是否在任何一个障碍物内部
        in_obstacle = False
        for obstacle in obstacles:
            if is_point_in_polygon(centroid, obstacle):
                in_obstacle = True
                break
        
        # 3. 如果重心不在障碍物内，保留三角形
        if not in_obstacle:
            filtered_tris.append(tri)
    
    return filtered_tris

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


# ---------- 新增：三角剖分（Bowyer-Watson）辅助函数 ----------
def _circumcircle(a, b, c):
    ax, ay = a; bx, by = b; cx, cy = c
    d = 2.0 * (ax*(by-cy) + bx*(cy-ay) + cx*(ay-by))
    if abs(d) < 1e-12:
        return (0.0, 0.0, float('inf'))
    a2 = ax*ax + ay*ay
    b2 = bx*bx + by*by
    c2 = cx*cx + cy*cy
    ux = (a2*(by-cy) + b2*(cy-ay) + c2*(ay-by)) / d
    uy = (a2*(cx-bx) + b2*(ax-cx) + c2*(bx-ax)) / d
    r2 = (ux-ax)**2 + (uy-ay)**2
    return (ux, uy, r2)

def _make_edge(a, b):
    # canonical order for edge key
    return (a, b) if a <= b else (b, a)

def triangulate_points(points):
    """
    Simple Bowyer-Watson Delaunay triangulation for a set of 2D points.
    Returns list of triangles, each triangle is ((x1,y1),(x2,y2),(x3,y3)).
    """
    if len(points) < 3:
        return []
    pts = list(points)

    # build super-triangle that contains all points
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    minx, maxx = min(xs), max(xs)
    miny, maxy = min(ys), max(ys)
    dx = maxx - minx; dy = maxy - miny
    dmax = max(dx, dy)
    midx = (minx + maxx) / 2.0
    midy = (miny + maxy) / 2.0
    st1 = (midx - 20*dmax, midy - dmax)
    st2 = (midx, midy + 20*dmax)
    st3 = (midx + 20*dmax, midy - dmax)
    pts.extend([st1, st2, st3])

    triangles = [ (st1, st2, st3) ]

    for p in pts[:-3]:
        bad = []
        circum_data = {}
        for tri in triangles:
            ux, uy, r2 = _circumcircle(tri[0], tri[1], tri[2])
            dxp = p[0] - ux; dyp = p[1] - uy
            if dxp*dxp + dyp*dyp <= r2 + 1e-9:
                bad.append(tri)
        # polygon hole boundary: collect edges of bad triangles that are not shared twice
        edge_count = {}
        for tri in bad:
            edges = [ _make_edge(tri[0], tri[1]), _make_edge(tri[1], tri[2]), _make_edge(tri[2], tri[0]) ]
            for e in edges:
                edge_count[e] = edge_count.get(e, 0) + 1
        boundary = [e for e,cnt in edge_count.items() if cnt == 1]
        # remove bad triangles
        triangles = [t for t in triangles if t not in bad]
        # re-triangulate hole with point p
        for e in boundary:
            triangles.append((e[0], e[1], p))

    # remove triangles that use super-triangle vertices
    result = []
    for tri in triangles:
        if st1 in tri or st2 in tri or st3 in tri:
            continue
        result.append(tri)
    return result

# ---------- end 新增 ----------

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

    # ---------- 新增：基于三角重心的局部规划函数（覆盖默认 planner） ----------
    def _point_in_triangle(p, a, b, c):
        # barycentric technique
        px, py = p; ax, ay = a; bx, by = b; cx, cy = c
        v0x, v0y = cx - ax, cy - ay
        v1x, v1y = bx - ax, by - ay
        v2x, v2y = px - ax, py - ay
        dot00 = v0x*v0x + v0y*v0y
        dot01 = v0x*v1x + v0y*v1y
        dot02 = v0x*v2x + v0y*v2y
        dot11 = v1x*v1x + v1y*v1y
        dot12 = v1x*v2x + v1y*v2y
        denom = dot00 * dot11 - dot01 * dot01
        if abs(denom) < 1e-12:
            return False
        u = (dot11 * dot02 - dot01 * dot12) / denom
        v = (dot00 * dot12 - dot01 * dot02) / denom
        return (u >= -1e-9) and (v >= -1e-9) and (u + v <= 1.0 + 1e-9)

    def _dist(a, b):
        return math.hypot(a[0]-b[0], a[1]-b[1])

    def _a_star_nodes(nodes, edges, src_idx, tgt_idx):
        import heapq
        N = len(nodes)
        g = [float('inf')] * N
        prev = [-1] * N
        g[src_idx] = 0.0
        def h(i):
            return _dist(nodes[i], nodes[tgt_idx])
        pq = [(h(src_idx), src_idx)]
        closed = [False] * N
        while pq:
            f,u = heapq.heappop(pq)
            if closed[u]:
                continue
            if u == tgt_idx:
                break
            closed[u] = True
            for v,w in edges.get(u, []):
                if closed[v]:
                    continue
                tentative = g[u] + w
                if tentative < g[v]:
                    g[v] = tentative
                    prev[v] = u
                    heapq.heappush(pq, (tentative + h(v), v))
        if g[tgt_idx] == float('inf'):
            return None
        path = []
        cur = tgt_idx
        while cur != -1:
            path.append(nodes[cur])
            cur = prev[cur]
        path.reverse()
        return path

    def plan_via_triangle_centroids(obstacles_py, starts_py, goals_py, steps=200):
        # Build point set: window corners + obstacle vertices
        w, h = screen.get_size()
        pts_set = set()
        pts_set.add((0.0, 0.0)); pts_set.add((float(w), 0.0))
        pts_set.add((float(w), float(h))); pts_set.add((0.0, float(h)))
        for poly in obstacles_py:
            for v in poly:
                pts_set.add((float(v[0]), float(v[1])))
        pts = list(pts_set)
        # triangulate
        try:
            tris = triangulate_points(pts)
            tris = filter_triangles_by_obstacles(tris, obstacles_py)
        except Exception:
            tris = []
        if not tris:
            # fallback: straight line paths
            res = []
            for s,g in zip(starts_py, goals_py):
                res.append([(s[0], s[1]), (g[0], g[1])])
            return res

        # centroids and edge->tri mapping
        centroids = []
        edge_map = {}  # edge_key -> list of tri indices
        for ti, tri in enumerate(tris):
            a,b,c = tri
            cx = (a[0]+b[0]+c[0]) / 3.0
            cy = (a[1]+b[1]+c[1]) / 3.0
            centroids.append((cx, cy))
            # edges as ordered keys
            edges = []
            for p,q in ((a,b),(b,c),(c,a)):
                key = (p, q) if p <= q else (q, p)
                edge_map.setdefault(key, []).append(ti)

        # build adjacency graph among centroids
        edges = {}
        for key, tri_idxs in edge_map.items():
            if len(tri_idxs) >= 2:
                for i in range(len(tri_idxs)):
                    for j in range(i+1, len(tri_idxs)):
                        u = tri_idxs[i]; v = tri_idxs[j]
                        wgt = _dist(centroids[u], centroids[v])
                        edges.setdefault(u, []).append((v, wgt))
                        edges.setdefault(v, []).append((u, wgt))

        results = []
        for s,g in zip(starts_py, goals_py):
            spt = (float(s[0]), float(s[1])); gpt = (float(g[0]), float(g[1]))
            # find triangle containing start/goal
            s_tri = None; g_tri = None
            for ti, tri in enumerate(tris):
                if s_tri is None and _point_in_triangle(spt, tri[0], tri[1], tri[2]):
                    s_tri = ti
                if g_tri is None and _point_in_triangle(gpt, tri[0], tri[1], tri[2]):
                    g_tri = ti
                if s_tri is not None and g_tri is not None:
                    break
            # if not found, pick nearest centroid
            if s_tri is None:
                s_tri = min(range(len(centroids)), key=lambda i: _dist(spt, centroids[i]))
            if g_tri is None:
                g_tri = min(range(len(centroids)), key=lambda i: _dist(gpt, centroids[i]))

            # create augmented nodes: centroids + start + goal
            nodes = list(centroids) + [spt, gpt]
            src_idx = len(nodes) - 2
            tgt_idx = len(nodes) - 1
            # clone edges and add connections from start/goal to their centroids
            aug_edges = {k: list(v) for k,v in edges.items()}
            # connect start node to its centroid
            aug_edges.setdefault(src_idx, []).append((s_tri, _dist(spt, centroids[s_tri])))
            aug_edges.setdefault(s_tri, []).append((src_idx, _dist(spt, centroids[s_tri])))
            # connect goal node to its centroid
            aug_edges.setdefault(tgt_idx, []).append((g_tri, _dist(gpt, centroids[g_tri])))
            aug_edges.setdefault(g_tri, []).append((tgt_idx, _dist(gpt, centroids[g_tri])))

            path = _a_star_nodes(nodes, aug_edges, src_idx, tgt_idx)
            if path is None:
                # fallback straight
                results.append([spt, gpt])
            else:
                # ensure strict following: return node path as-is (centroids sequence includes start/goal)
                results.append(path)
        return results

    # 覆盖原先的 planner，强制使用三角重心规划
    plan_paths_fn = plan_via_triangle_centroids
    # ---------- 结束新增覆盖 ----------

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

        # per-frame computation: incremental replanning to limit work per frame
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
            subset_starts = [current_starts[i] for i in indices]
            subset_goals = [goals[i] for i in indices]
            try:
                subset_paths = plan_paths_fn(obstacles, subset_starts, subset_goals, steps=plan_step_max)
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

            # 如果已非常接近最近的导航点，则认为已到达并从路径中移除该点，避免停在点上
            if path:
                near_dist = math.hypot(path[nearest_idx][0] - drone.pos[0], path[nearest_idx][1] - drone.pos[1])
                # 判断：当无人机中心与导航点距离小于无人机半径时视为到达
                if near_dist-10 < drone.radius:
                    try:
                        del path[nearest_idx]
                    except Exception:
                        pass
                    # 同步回存储的路径
                    drone.nav_points = path
                    planned_paths[i] = path
                    if not path:
                        continue

            # reached goal check
            spos = (int(drone.pos[0]*SCALE + OFFSET[0]), int(drone.pos[1]*SCALE + OFFSET[1]))
            goal_world = goals[i]
            gpos = (int(goal_world[0]*SCALE + OFFSET[0]), int(goal_world[1]*SCALE + OFFSET[1]))
            if (not drone.arrived) and (math.hypot(spos[0]-gpos[0], spos[1]-gpos[1]) <= 25):
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
                # 使用最近起的最多 6 个导航点进行加权导航（权重为 1,0.5,0.25,...），
                # 再取加权平均作为目标向量。这比单点前瞻更平滑且兼容不同采样密度。
                if not path:
                    continue
                # 防御性保证 nearest_idx 在当前 path 范围内
                if nearest_idx >= len(path):
                    nearest_idx = 0
                    best_d2 = float('inf')
                    for idx, p in enumerate(path):
                        dx = p[0] - drone.pos[0]
                        dy = p[1] - drone.pos[1]
                        d2 = dx*dx + dy*dy
                        if d2 < best_d2:
                            best_d2 = d2
                            nearest_idx = idx
                path=path[nearest_idx:]
                nearest_idx = 0
                max_k = 6
                ws = []
                vec_sum = [0.0, 0.0]
                wsum = 0.0
                for k in range(max_k):
                    idxk = nearest_idx + k
                    if idxk >= len(path):
                        break
                    pk = path[idxk]
                    vk = (pk[0] - drone.pos[0], pk[1] - drone.pos[1])
                    wk = (k==0)
                    vec_sum[0] += wk * vk[0]
                    vec_sum[1] += wk * vk[1]
                    wsum += wk
                if wsum == 0.0:
                    target_vec = (0.0, 0.0)
                else:
                    target_vec = (vec_sum[0] / wsum, vec_sum[1] / wsum)

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
        w,h = screen.get_size()
        # collect unique points: window corners + obstacle vertices
        pts_set = set()
        # window corners
        pts_set.add((0.0, 0.0)); pts_set.add((float(w), 0.0))
        pts_set.add((float(w), float(h))); pts_set.add((0.0, float(h)))
        for poly in obstacles:
            for v in poly:
                pts_set.add((float(v[0]), float(v[1])))
        
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
            "tris": filter_triangles_by_obstacles(triangulate_points(list(pts_set)), obstacles)
        }

        # provide current FPS to renderer (clock.get_fps may reflect recent frames)
        state["fps"] = clock.get_fps()
        renderer.draw_frame(screen, font, state)

        # ---------- 新增：计算并绘制三角剖分（窗口顶点 + 障碍物顶点） ----------
        clock.tick(60)
        
    pygame.quit()


if __name__ == "__main__":
    main()
