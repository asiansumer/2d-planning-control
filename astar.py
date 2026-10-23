"""格子地图上的 A* 搜索（四邻居，每步代价 1，曼哈顿距离作启发函数）。

格子用 (x, y) 元组表示，blocked 是墙格子的集合。
"""


def neighbors(cell, blocked, width, height):
    """返回 cell 上下左右四个邻居中能走的那些：在地图范围内且不是墙。"""
    x, y = cell
    result = []
    for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < width and 0 <= ny < height:
            if (nx, ny) not in blocked:
                result.append((nx, ny))
    return result


def reconstruct_path(came_from, start, goal):
    """从 goal 顺着 came_from 一路往回找到 start，返回从起点到终点的路线。

    came_from[c] 记录的是“c 是从哪个格子走过来的”。
    """
    path = [goal]
    cell = goal
    while cell != start:
        cell = came_from[cell]
        path.append(cell)
    path.reverse()
    return path


def heuristic(cell, goal):
    """曼哈顿距离：横向差几格 + 纵向差几格，估计到终点还要走几步。"""
    return abs(goal[0] - cell[0]) + abs(goal[1] - cell[1])


def astar(start, goal, blocked, width, height):
    """从 start 搜索到 goal，返回路线（格子列表）；无路可走时返回 None。"""
    # open_set：待考察的格子；g：从起点到该格子的已知最少步数；came_from：每个格子的来处
    open_set = [start]
    g = {start: 0}
    came_from = {}

    while len(open_set) > 0:
        # 取出 f = g + h 最小的格子
        current = open_set[0]
        for cell in open_set:
            f_cell = g[cell] + heuristic(cell, goal)
            f_current = g[current] + heuristic(current, goal)
            if f_cell < f_current:
                current = cell

        open_set.remove(current)
        # 在取出时才判断终点，这样保证拿到的是最短路线
        if current == goal:
            return reconstruct_path(came_from, start, goal)

        for nb in neighbors(current, blocked, width, height):
            new_g = g[current] + 1
            # 第一次到达这个邻居，或者找到了一条更近的路
            if nb not in g or new_g < g[nb]:
                g[nb] = new_g
                came_from[nb] = current
                if nb not in open_set:
                    open_set.append(nb)

    return None
