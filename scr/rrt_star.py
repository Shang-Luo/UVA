"""
RRT* 简单实现（仅包含实现用的函数）

接口说明（简要）:
- `rrt_star(start, goal, obstacles, bounds, max_iter=500, step_size=1.0, search_radius=5.0, goal_sample_rate=0.05)`
    - `start`/`goal`: (x, y)
    - `obstacles`: 列表，支持两种格式:
        * 圆: (x, y, r) 或 {'type':'circle','center':(x,y),'r':r}
        * 轴对齐矩形: {'type':'rect','xmin':..., 'ymin':..., 'xmax':..., 'ymax':...}
    - `bounds`: (xmin, ymin, xmax, ymax)

返回值: 路径为点列表 [(x,y), ...]；找不到路径时返回 None

注意: 这是一个轻量实现，旨在用于教学和集成。碰撞检测仅含基本圆/矩形检测。
"""

import math
import random
from typing import List, Tuple, Optional, Dict, Any

Point = Tuple[float, float]
Obstacle = Any


def _dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def nearest(nodes: List[Dict], point: Point) -> int:
    """返回 nodes 中距离 point 最近的节点索引。"""
    best_idx = 0
    best_d = float('inf')
    for i, n in enumerate(nodes):
        d = _dist(n['point'], point)
        if d < best_d:
            best_d = d
            best_idx = i
    return best_idx


def steer(from_pt: Point, to_pt: Point, step_size: float) -> Point:
    """从 from_pt 朝 to_pt 前进不超过 step_size，返回新点。"""
    d = _dist(from_pt, to_pt)
    if d <= step_size:
        return to_pt
    theta = math.atan2(to_pt[1] - from_pt[1], to_pt[0] - from_pt[0])
    return (from_pt[0] + step_size * math.cos(theta), from_pt[1] + step_size * math.sin(theta))


def _point_in_rect(p: Point, rect: Dict) -> bool:
    return rect['xmin'] <= p[0] <= rect['xmax'] and rect['ymin'] <= p[1] <= rect['ymax']


def _seg_point_distance(a: Point, b: Point, p: Point) -> float:
    # distance from point p to segment ab
    ax, ay = a
    bx, by = b
    px, py = p
    dx = bx - ax
    dy = by - ay
    if dx == dy == 0:
        return _dist(a, p)
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    proj = (ax + t * dx, ay + t * dy)
    return _dist(proj, p)


def _seg_intersect(a1: Point, a2: Point, b1: Point, b2: Point) -> bool:
    # check segment intersection (2D) using orientation
    def orient(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])

    o1 = orient(a1, a2, b1)
    o2 = orient(a1, a2, b2)
    o3 = orient(b1, b2, a1)
    o4 = orient(b1, b2, a2)

    if o1 == o2 == o3 == o4 == 0:
        # colinear - fallback to bbox overlap
        def on_seg(a,b,c):
            return min(a[0],b[0]) <= c[0] <= max(a[0],b[0]) and min(a[1],b[1]) <= c[1] <= max(a[1],b[1])
        return on_seg(a1,a2,b1) or on_seg(a1,a2,b2) or on_seg(b1,b2,a1) or on_seg(b1,b2,a2)
    return (o1*o2 <= 0) and (o3*o4 <= 0)


def collision_free(a: Point, b: Point, obstacles: List[Obstacle], robot_radius: float = 0.0) -> bool:
    """检查线段 ab 与 obstacles 是否无碰撞。支持圆和轴对齐矩形。
    参数 `robot_radius` 会膨胀障碍物（等价于将机器人视为点、障碍物膨胀）。
    """
    for obs in obstacles:
        # circle as tuple (x,y,r)
        if isinstance(obs, tuple) and len(obs) == 3:
            cx, cy, r = obs
            if _seg_point_distance(a, b, (cx, cy)) <= (r + robot_radius):
                return False
        elif isinstance(obs, dict):
            t = obs.get('type')
            if t == 'circle':
                cx, cy = obs['center']
                r = obs['r']
                if _seg_point_distance(a, b, (cx, cy)) <= (r + robot_radius):
                    return False
            elif t == 'rect':
                # inflate rectangle by robot_radius
                rxmin = obs['xmin'] - robot_radius
                rymin = obs['ymin'] - robot_radius
                rxmax = obs['xmax'] + robot_radius
                rymax = obs['ymax'] + robot_radius
                inflated = {'xmin': rxmin, 'ymin': rymin, 'xmax': rxmax, 'ymax': rymax}
                # check if either endpoint inside inflated rect
                if _point_in_rect(a, inflated) or _point_in_rect(b, inflated):
                    return False
                # check intersection with rectangle edges
                rect_edges = [
                    ((rxmin, rymin), (rxmax, rymin)),
                    ((rxmax, rymin), (rxmax, rymax)),
                    ((rxmax, rymax), (rxmin, rymax)),
                    ((rxmin, rymax), (rxmin, rymin)),
                ]
                for e1, e2 in rect_edges:
                    if _seg_intersect(a, b, e1, e2):
                        return False
            else:
                # unknown dict format: try circle-like
                if 'center' in obs and 'r' in obs:
                    cx, cy = obs['center']
                    r = obs['r']
                    if _seg_point_distance(a, b, (cx, cy)) <= (r + robot_radius):
                        return False
    return True


def near(nodes: List[Dict], point: Point, radius: float) -> List[int]:
    idxs = []
    for i, n in enumerate(nodes):
        if _dist(n['point'], point) <= radius:
            idxs.append(i)
    return idxs


def choose_parent(nodes: List[Dict], new_pt: Point, near_idxs: List[int], obstacles: List[Obstacle], robot_radius: float = 0.0) -> Tuple[int, float]:
    best_parent = -1
    best_cost = float('inf')
    for i in near_idxs:
        n = nodes[i]
        if collision_free(n['point'], new_pt, obstacles, robot_radius):
            cost = n['cost'] + _dist(n['point'], new_pt)
            if cost < best_cost:
                best_cost = cost
                best_parent = i
    return best_parent, best_cost


def rewire(nodes: List[Dict], new_idx: int, near_idxs: List[int], obstacles: List[Obstacle], robot_radius: float = 0.0):
    new_node = nodes[new_idx]
    for i in near_idxs:
        if i == new_idx:
            continue
        n = nodes[i]
        if collision_free(new_node['point'], n['point'], obstacles, robot_radius):
            new_cost = new_node['cost'] + _dist(new_node['point'], n['point'])
            if new_cost < n['cost']:
                nodes[i]['parent'] = new_idx
                nodes[i]['cost'] = new_cost


def get_path(nodes: List[Dict], goal_idx: int) -> List[Point]:
    path = []
    cur = goal_idx
    while cur is not None:
        path.append(nodes[cur]['point'])
        cur = nodes[cur]['parent']
    path.reverse()
    return path


def _bezier_point(b0: Point, b1: Point, b2: Point, b3: Point, t: float) -> Point:
    u = 1.0 - t
    uu = u * u
    uuu = uu * u
    tt = t * t
    ttt = tt * t
    x = uuu * b0[0] + 3 * uu * t * b1[0] + 3 * u * tt * b2[0] + ttt * b3[0]
    y = uuu * b0[1] + 3 * uu * t * b1[1] + 3 * u * tt * b2[1] + ttt * b3[1]
    return (x, y)


def smooth_path_bezier(path: List[Point], samples_per_segment: int = 8) -> List[Point]:
    """使用 Catmull-Rom -> Cubic Bezier 转换平滑路径。

    方法：对每个相邻点对 P1->P2，使用 P0（前）和 P3（后）作为邻点，
    将该段转换为一段三次 Bézier：
      B0 = P1
      B1 = P1 + (P2 - P0) / 6
      B2 = P2 - (P3 - P1) / 6
      B3 = P2
    对每段按 samples_per_segment 采样并拼接，保持连续性。
    """
    if not path:
        return []
    n = len(path)
    if n == 1:
        return [path[0]]
    if n == 2:
        # linear interpolation between two points
        p0, p1 = path[0], path[1]
        res = []
        for i in range(samples_per_segment + 1):
            t = i / float(samples_per_segment)
            res.append((p0[0] * (1 - t) + p1[0] * t, p0[1] * (1 - t) + p1[1] * t))
        return res

    smoothed: List[Point] = []
    for i in range(n - 1):
        p0 = path[i - 1] if i - 1 >= 0 else path[i]
        p1 = path[i]
        p2 = path[i + 1]
        p3 = path[i + 2] if i + 2 < n else path[i + 1]

        # compute Bezier control points from Catmull-Rom
        b0 = p1
        b1 = (p1[0] + (p2[0] - p0[0]) / 6.0, p1[1] + (p2[1] - p0[1]) / 6.0)
        b2 = (p2[0] - (p3[0] - p1[0]) / 6.0, p2[1] - (p3[1] - p1[1]) / 6.0)
        b3 = p2

        # sample segment (avoid duplicating last point)
        for s in range(samples_per_segment):
            t = s / float(samples_per_segment)
            smoothed.append(_bezier_point(b0, b1, b2, b3, t))
    # append final point
    smoothed.append(path[-1])
    return smoothed


def rrt_star(start: Point,
             goal: Point,
             obstacles: List[Obstacle],
             bounds: Tuple[float, float, float, float],
             max_iter: int = 500,
             step_size: float = 1.0,
             search_radius: float = 5.0,
             goal_sample_rate: float = 0.05,
             robot_radius: float = 0.0,
             smooth: bool = False,
             samples_per_segment: int = 8) -> Optional[List[Point]]:
    """执行 RRT* 搜索，返回从 start 到 goal 的路径点列表或 None。"""
    xmin, ymin, xmax, ymax = bounds

    def sample() -> Point:
        if random.random() < goal_sample_rate:
            return goal
        return (random.uniform(xmin, xmax), random.uniform(ymin, ymax))

    nodes: List[Dict] = []
    nodes.append({'point': start, 'parent': None, 'cost': 0.0})

    goal_reached_idx = None
    for it in range(max_iter):
        rnd = sample()
        nid = nearest(nodes, rnd)
        new_pt = steer(nodes[nid]['point'], rnd, step_size)
        if not collision_free(nodes[nid]['point'], new_pt, obstacles, robot_radius):
            continue

        near_idxs = near(nodes, new_pt, search_radius)
        parent_idx, cost = choose_parent(nodes, new_pt, near_idxs + [nid], obstacles, robot_radius)
        if parent_idx == -1:
            parent_idx = nid
            cost = nodes[nid]['cost'] + _dist(nodes[nid]['point'], new_pt)

        new_node = {'point': new_pt, 'parent': parent_idx, 'cost': cost}
        nodes.append(new_node)
        new_idx = len(nodes) - 1

        # rewire
        rewire(nodes, new_idx, near_idxs, obstacles, robot_radius)

        # check goal
        if _dist(new_pt, goal) <= step_size and collision_free(new_pt, goal, obstacles, robot_radius):
            # add goal node
            goal_cost = nodes[new_idx]['cost'] + _dist(new_pt, goal)
            nodes.append({'point': goal, 'parent': new_idx, 'cost': goal_cost})
            goal_reached_idx = len(nodes) - 1
            break

    if goal_reached_idx is None:
        # try to connect nearest node to goal (post-processing)
        nid = nearest(nodes, goal)
        if collision_free(nodes[nid]['point'], goal, obstacles, robot_radius):
            nodes.append({'point': goal, 'parent': nid, 'cost': nodes[nid]['cost'] + _dist(nodes[nid]['point'], goal)})
            goal_reached_idx = len(nodes) - 1

    if goal_reached_idx is None:
        return None
    path = get_path(nodes, goal_reached_idx)
    if smooth and path:
        try:
            path = smooth_path_bezier(path, samples_per_segment=samples_per_segment)
        except Exception:
            pass
    return path
