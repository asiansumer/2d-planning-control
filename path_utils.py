import math

from geometry import segment_collision


def path_length(path):
    """路线总长：把相邻两点的距离加起来。"""
    length = 0.0
    for a, b in zip(path, path[1:]):
        length += math.hypot(b[0] - a[0], b[1] - a[1])
    return length


def shortcut(path, obstacles):
    """拉直路线：从当前点出发，找最远的、能直接连过去不撞的点，中间的点全部跳过。"""
    result = [path[0]]
    i = 0
    while i < len(path) - 1:
        # 从最后一个点开始往回试，直到找到能直连的点
        j = len(path) - 1
        while j > i + 1 and segment_collision(path[i], path[j], obstacles):
            j -= 1
        result.append(path[j])
        i = j
    return result


def densify(path, ds=0.2):
    """加密路线：相邻两点之间每隔 ds 米插一个点，让 Pure Pursuit 有足够多的点可以瞄。"""
    result = [path[0]]
    for a, b in zip(path, path[1:]):
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, math.ceil(dist / ds))
        for k in range(1, n + 1):
            t = k / n
            result.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
    return result


def point_to_segment(p, a, b):
    """点 p 到线段 a→b 的最短距离。"""
    abx, aby = b[0] - a[0], b[1] - a[1]
    seg_len2 = abx * abx + aby * aby
    if seg_len2 == 0:
        return math.hypot(p[0] - a[0], p[1] - a[1])
    # t：p 在线段上的投影位置，0 是 a，1 是 b，超出范围就夹到 0 或 1
    t = ((p[0] - a[0]) * abx + (p[1] - a[1]) * aby) / seg_len2
    t = max(0.0, min(1.0, t))
    px, py = a[0] + t * abx, a[1] + t * aby
    return math.hypot(p[0] - px, p[1] - py)


def distance_to_path(x, y, path):
    """横向误差：车到整条路线的最短距离（找最近的点，再看它前后两段线段）。"""
    best_i = 0
    best_d = float("inf")
    for i, (px, py) in enumerate(path):
        d = math.hypot(px - x, py - y)
        if d < best_d:
            best_i = i
            best_d = d
    for k in [best_i - 1, best_i]:
        if 0 <= k < len(path) - 1:
            best_d = min(best_d, point_to_segment((x, y), path[k], path[k + 1]))
    return best_d
