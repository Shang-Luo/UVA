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
    # precompute extended polygon vertices:
    # 对于每个障碍物顶点，在该点处沿外角平分线向外延伸一定距离（30px），并把延伸点作为可见图节点。
    # 注意：起点和终点不做延伸，仍在后面单独加入。
    poly_vertices = []
    extend_dist = 30.0
    for poly in obstacles:
        if len(poly) == 0:
            continue
        c = _centroid(poly)
        n = len(poly)
        for i, v in enumerate(poly):
            p = poly[(i-1) % n]
            nn = poly[(i+1) % n]
            # 单位向量从顶点指向相邻顶点
            u_prev = _normalize((p[0]-v[0], p[1]-v[1]))
            u_next = _normalize((nn[0]-v[0], nn[1]-v[1]))
            bx = u_prev[0] + u_next[0]
            by = u_prev[1] + u_next[1]
            b_norm = math.hypot(bx, by)
            if b_norm < 1e-6:
                # 若两邻边几乎共线或角度接近 180°，退化到从质心指向顶点的方向
                od = _normalize((v[0]-c[0], v[1]-c[1]))
            else:
                bx /= b_norm
                by /= b_norm
                # interior bisector = (u_prev+u_next); 根据其与顶点指向质心向量的点积确定朝向，确保得到外侧方向
                dot = bx*(v[0]-c[0]) + by*(v[1]-c[1])
                if dot < 0:
                    od = (-bx, -by)
                else:
                    od = (bx, by)
            ext = (v[0] + od[0]*extend_dist, v[1] + od[1]*extend_dist)
            poly_vertices.append(ext)

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

        # sample the vertex path into dense points
        sampled = sample_polyline(vg_path, steps)
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
    k_goal = params.get("k_goal", 0.6)#目标吸引力增益
    avoid_gain = params.get("avoid_gain", 1.8)#障碍物排斥力增益
    max_acc = params.get("max_acc", 5.0)#最大加速度
    max_range = params.get("max_range", max(radar_readings) if radar_readings else 1.0)#雷达最大有效范围

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
