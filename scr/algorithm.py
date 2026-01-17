import math

def _seg_seg_intersect(a1, a2, b1, b2):
    # 检查线段 a1-a2 与 b1-b2 是否相交（严格相交）
    def orient(p, q, r):
        return (q[0]-p[0])*(r[1]-p[1]) - (q[1]-p[1])*(r[0]-p[0])
    o1 = orient(a1, a2, b1)
    o2 = orient(a1, a2, b2)
    o3 = orient(b1, b2, a1)
    o4 = orient(b1, b2, a2)
    return o1*o2 < 0 and o3*o4 < 0

def _line_intersects_poly(p1, p2, poly):
    for i in range(len(poly)):
        q1 = poly[i]
        q2 = poly[(i+1)%len(poly)]
        if _seg_seg_intersect(p1, p2, q1, q2):
            return True
    return False


def _point_in_poly(pt, poly):
    # ray casting algorithm (works for non-self-intersecting polygons)
    x, y = pt
    inside = False
    n = len(poly)
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[(i+1) % n]
        # check if edge crosses horizontal ray to the right of pt
        intersect = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi
        )
        if intersect:
            inside = not inside
    return inside


def _segment_crosses_poly(p1, p2, poly):
    # True if segment intersects polygon edges or one endpoint is inside polygon
    if _point_in_poly(p1, poly) or _point_in_poly(p2, poly):
        return True
    return _line_intersects_poly(p1, p2, poly)

def _centroid(poly):
    x = sum(p[0] for p in poly) / len(poly)
    y = sum(p[1] for p in poly) / len(poly)
    return (x, y)

def _normalize(v):
    n = math.hypot(v[0], v[1]) or 1.0
    return (v[0]/n, v[1]/n)

def _interp(a, b, t):
    return (a[0] + (b[0]-a[0])*t, a[1] + (b[1]-a[1])*t)

def plan_paths(obstacles, starts, goals, steps=200):
    """
    使用可见图 (visibility graph) 为每对 start->goal 规划路径。
    返回每架无人机的路径点列表（2D）。
    算法：
    1. 节点集合 = 所有多边形顶点 + start + goal
    2. 对每对节点 (A,B)：若线段 AB 不穿过任何障碍内部且在边相交时只允许端点重合，则加入图边
    3. 在图上使用 Dijkstra 找最短路径（欧氏距离权重）
    4. 将顶点序列按段插值为总共约 `steps` 个点返回
    """
    import heapq

    def dist(a, b):
        return math.hypot(a[0]-b[0], a[1]-b[1])

    def visible(a, b, obstacles):
        # 允许端点位于多边形边上或顶点；不允许线段穿过多边形内部或与边在非端点处相交
        # 快速中点内点检测：若中点在任一多边形内部，则不可见
        mid = ((a[0]+b[0])/2.0, (a[1]+b[1])/2.0)
        for poly in obstacles:
            if _point_in_poly(mid, poly):
                return False
        # 检查与每条边的严格相交（非共享端点）
        for poly in obstacles:
            n = len(poly)
            for i in range(n):
                u = poly[i]
                v = poly[(i+1)%n]
                if _seg_seg_intersect(a, b, u, v):
                    # 如果相交点是端点重合（a==u or a==v or b==u or b==v），则允许
                    if ( (a[0]==u[0] and a[1]==u[1]) or (a[0]==v[0] and a[1]==v[1]) or
                         (b[0]==u[0] and b[1]==u[1]) or (b[0]==v[0] and b[1]==v[1]) ):
                        continue
                    return False
        return True

    def dijkstra(nodes, edges, src_idx, tgt_idx):
        N = len(nodes)
        dist_arr = [float('inf')] * N
        prev = [-1] * N
        dist_arr[src_idx] = 0.0
        pq = [(0.0, src_idx)]
        while pq:
            d,u = heapq.heappop(pq)
            if d > dist_arr[u]:
                continue
            if u == tgt_idx:
                break
            for v,w in edges.get(u, []):
                nd = d + w
                if nd < dist_arr[v]:
                    dist_arr[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))
        if dist_arr[tgt_idx] == float('inf'):
            return None
        # reconstruct
        path = []
        cur = tgt_idx
        while cur != -1:
            path.append(nodes[cur])
            cur = prev[cur]
        path.reverse()
        return path

    def sample_polyline(pts, steps):
        # 将多段折线采样为近似 steps 个点，按段长度分配
        if len(pts) == 0:
            return []
        seg_lens = []
        total = 0.0
        for i in range(len(pts)-1):
            l = dist(pts[i], pts[i+1])
            seg_lens.append(l)
            total += l
        if total == 0:
            return [pts[0]] * steps
        samples = []
        for i in range(len(pts)-1):
            a = pts[i]; b = pts[i+1]
            # allocate k samples proportional to length
            k = max(1, int(round(seg_lens[i] / total * steps)))
            for j in range(k):
                t = j / float(k)
                samples.append(_interp(a, b, t))
        samples.append(pts[-1])
        # 如果采样点多于 steps，则截断到前 steps 个；若少于 steps，保持原样（不再填充）
        if len(samples) > steps:
            return samples[:steps]
        return samples

    paths = []
    # precompute polygon vertices
    poly_vertices = []
    for poly in obstacles:
        for v in poly:
            poly_vertices.append((v[0], v[1]))

    for s, g in zip(starts, goals):
        # if straight segment is free, use it
        if visible(tuple(s), tuple(g), obstacles):
            path = [_interp(s, g, i/(steps-1)) for i in range(steps)]
            paths.append(path)
            continue

        # build nodes: polygon vertices + s + g
        nodes = list(poly_vertices) + [(s[0], s[1]), (g[0], g[1])]
        src_idx = len(nodes) - 2
        tgt_idx = len(nodes) - 1

        # build edges
        edges = {}
        N = len(nodes)
        for i in range(N):
            for j in range(i+1, N):
                a = nodes[i]; b = nodes[j]
                if visible(a, b, obstacles):
                    w = dist(a, b)
                    edges.setdefault(i, []).append((j, w))
                    edges.setdefault(j, []).append((i, w))

        vg_path = dijkstra(nodes, edges, src_idx, tgt_idx)
        if vg_path is None:
            # no path found: fallback to straight interpolation
            path = [_interp(s, g, i/(steps-1)) for i in range(steps)]
            paths.append(path)
            continue

              # ===== 新增：对于中间的多边形顶点，沿外角平分线外移 30px =====
        # 构建顶点->(poly_idx, idx_in_poly) 映射以查找邻接顶点
        vert_map = {}
        for pi, poly in enumerate(obstacles):
            n = len(poly)
            for vi in range(n):
                key = (float(poly[vi][0]), float(poly[vi][1]))
                vert_map.setdefault(key, []).append((pi, vi))

        def _safe_normalize(v):
            nx = v[0]; ny = v[1]
            m = math.hypot(nx, ny)
            if m < 1e-9:
                return (0.0, 0.0)
            return (nx/m, ny/m)

        EXT_LEN = 30.0
        adjusted_path = []
        for idx_p, pt in enumerate(vg_path):
            # 保留首尾点不变
            if idx_p == 0 or idx_p == len(vg_path)-1:
                adjusted_path.append(pt)
                continue
            key = (float(pt[0]), float(pt[1]))
            if key not in vert_map:
                adjusted_path.append(pt)
                continue
            # 若顶点出现在多个多边形，优先选第一个匹配（通常不会有歧义）
            poly_idx, vi = vert_map[key][0]
            poly = obstacles[poly_idx]
            n = len(poly)
            # 相邻顶点 u (prev) 和 w (next)
            u = (poly[(vi-1)%n][0], poly[(vi-1)%n][1])
            w = (poly[(vi+1)%n][0], poly[(vi+1)%n][1])
            v = (pt[0], pt[1])
            vu = (u[0]-v[0], u[1]-v[1])
            vw = (w[0]-v[0], w[1]-v[1])
            nu = _safe_normalize(vu)
            nw = _safe_normalize(vw)
            # 内角平分线（方向向内）
            bis = (nu[0] + nw[0], nu[1] + nw[1])
            bis = _safe_normalize(bis)
            if bis == (0.0, 0.0):
                adjusted_path.append(pt)
                continue
            # 外角平分线候选方向（取内角平分线的相反方向）
            cand = (-bis[0], -bis[1])
            # 判断 cand 是否指向多边形外侧：质心->顶点 向量 与 cand 的点积应为正
            cent = _centroid(poly)
            vec_cv = (v[0] - cent[0], v[1] - cent[1])
            dot = vec_cv[0]*cand[0] + vec_cv[1]*cand[1]
            if dot >= 0:
                ext_dir = _safe_normalize(cand)
            else:
                # 若不符合则取相反方向（保证朝外）
                ext_dir = _safe_normalize((-cand[0], -cand[1]))
            # 延长
            new_pt = (v[0] + ext_dir[0]*EXT_LEN, v[1] + ext_dir[1]*EXT_LEN)
            adjusted_path.append(new_pt)
        # 用调整后的顶点序列继续采样
        sampled = sample_polyline(adjusted_path, steps)
        paths.append(sampled)

    return paths


def compute_acceleration(radar_readings, directions, target_vec, vel, params=None):
    """
    Simple reactive controller:
    - radar_readings: list of distances for each direction (length N)
    - directions: list of unit vectors for each radar direction
    - target_vec: tuple (dx,dy) pointing from current pos to target point
    - vel: current velocity (vx,vy)
    Returns: (ax, ay)
    """
    if params is None:
        params = {}
    # tunable gains
    k_goal = params.get("k_goal", 0.6)
    avoid_gain = params.get("avoid_gain", 1.8)
    max_acc = params.get("max_acc", 5.0)
    max_range = params.get("max_range", max(radar_readings) if radar_readings else 1.0)

    # desired direction towards target
    tx, ty = target_vec
    tnorm = math.hypot(tx, ty)
    if tnorm > 1e-6:
        tdir = (tx/tnorm, ty/tnorm)
    else:
        tdir = (0.0, 0.0)

    # goal component
    ax = k_goal * tdir[0]
    ay = k_goal * tdir[1]

    # repulsive from obstacles based on radar
    # repulsive from obstacles based on radar (inverse-square law)
    eps = 1e-3
    for d, ddir in zip(radar_readings, directions):
        if d <= 0:
            continue
        # only consider within max_range
        if d <= max_range:
            # inverse-square repulsion; larger when closer
            strength = avoid_gain * (1.0 / ((d + eps) ** 2))
            ax += -ddir[0] * strength
            ay += -ddir[1] * strength

    # apply simple damping to account for current velocity
    ax += -0.2 * vel[0]
    ay += -0.2 * vel[1]

    # cap acceleration
    m = math.hypot(ax, ay)
    if m > max_acc and m > 0:
        s = max_acc / m
        ax *= s
        ay *= s

    return (ax, ay)
