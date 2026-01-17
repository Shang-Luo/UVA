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

def plan_paths(obstacles, starts, goals, steps=200, grid=None):
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

    # grid-based A* planning will be attempted below after polyline sampler is defined

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

    # If a grid (from run.py subdivide_grid) is provided, perform A* on that grid.
    if grid is not None:
        # grid: list of (x,y,size)
        cells = list(grid)
        # build rects and centers
        rects = []
        centers = []
        for (x, y, size) in cells:
            rects.append((x, y, x+size, y+size))
            centers.append((x + size/2.0, y + size/2.0))

        # adjacency: two cells are adjacent if they share an edge (overlap on the other axis)
        N = len(cells)
        neighbors = [[] for _ in range(N)]
        eps = 1e-6
        for i in range(N):
            ax1, ay1, ax2, ay2 = rects[i]
            for j in range(i+1, N):
                bx1, by1, bx2, by2 = rects[j]
                # check horizontal touch
                horiz_touch = (abs(ax2 - bx1) < eps or abs(bx2 - ax1) < eps) and not (ay2 <= by1 + eps or by2 <= ay1 + eps)
                vert_touch = (abs(ay2 - by1) < eps or abs(by2 - ay1) < eps) and not (ax2 <= bx1 + eps or bx2 <= ax1 + eps)
                if horiz_touch or vert_touch:
                    # weight by center distance
                    w = math.hypot(centers[i][0]-centers[j][0], centers[i][1]-centers[j][1])
                    neighbors[i].append((j, w))
                    neighbors[j].append((i, w))

        def find_cell_for_point(p):
            x, y = p[0], p[1]
            for idx, (rx1, ry1, rx2, ry2) in enumerate(rects):
                if rx1 <= x <= rx2 and ry1 <= y <= ry2:
                    return idx
            # fallback: nearest center
            best = 0
            bd = float('inf')
            for i, c in enumerate(centers):
                d = math.hypot(c[0]-x, c[1]-y)
                if d < bd:
                    bd = d
                    best = i
            return best

        def astar(start_idx, goal_idx):
            openq = []
            heapq.heappush(openq, (0.0, start_idx))
            gscore = {start_idx: 0.0}
            prev = {start_idx: -1}
            while openq:
                f,u = heapq.heappop(openq)
                if u == goal_idx:
                    # reconstruct
                    path = []
                    cur = u
                    while cur != -1:
                        path.append(cur)
                        cur = prev.get(cur, -1)
                    path.reverse()
                    return path
                for v,w in neighbors[u]:
                    ng = gscore[u] + w
                    if ng < gscore.get(v, float('inf')):
                        gscore[v] = ng
                        prev[v] = u
                        # heuristic: euclidean from v to goal center
                        h = math.hypot(centers[v][0]-centers[goal_idx][0], centers[v][1]-centers[goal_idx][1])
                        heapq.heappush(openq, (ng + h, v))
            return None

        paths = []
        for s, g in zip(starts, goals):
            si = find_cell_for_point(s)
            gi = find_cell_for_point(g)
            pidx = astar(si, gi)
            if pidx is None:
                # fallback to visibility graph method below
                break
            # convert indices to centers sequence
            pts = [centers[i] for i in pidx]
            sampled = sample_polyline(pts, steps)
            paths.append(sampled)
        else:
            return paths
        # if any pair failed (no path), fall through to original VG method
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
