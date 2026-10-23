"""路线工具：长度、拉直、加密、横向误差。路线是 [(x, y), ...] 点列表。"""
import math

from geometry import segment_collision


def path_length(path):
    """路线总长：相邻两点距离之和。"""
    length = 0.0
    for a, b in zip(path, path[1:]):
        length += math.hypot(b[0] - a[0], b[1] - a[1])
    return length


def shortcut(path, obstacles):
    """拉直路线：从当前点出发，直接连到最远的、连线不撞障碍物的点，跳过中间的点。"""
    result = [path[0]]
    i = 0
    while i < len(path) - 1:
        # 从最后一个点往回试，第一个能直连的就是最远的
        j = len(path) - 1
        while j > i + 1 and segment_collision(path[i], path[j], obstacles):
            j -= 1
        result.append(path[j])
        i = j
    return result


def densify(path, ds=0.2):
    """加密路线：相邻两点之间每隔 ds 米插一个点，让 Pure Pursuit 有足够密的点可以瞄。"""
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
    # t 是 p 在线段上的投影位置：0 对应 a，1 对应 b，超出范围就截到 [0, 1]
    t = ((p[0] - a[0]) * abx + (p[1] - a[1]) * aby) / seg_len2
    t = max(0.0, min(1.0, t))
    px, py = a[0] + t * abx, a[1] + t * aby
    return math.hypot(p[0] - px, p[1] - py)


def distance_to_path(x, y, path):
    """横向误差：(x, y) 到整条路线的最短距离。

    先找最近的路点，再算到它前后两段线段的距离，取最小值。
    """
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
