def neighbors(cell, blocked, width, height):
    """返回格子 cell 上下左右四个邻居中，能走的那些（在地图内、不是墙）。"""
    x, y = cell
    result = []
    for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:   # 右、左、上、下
        nx, ny = x + dx, y + dy
        if 0 <= nx < width and 0<= ny < height:
            if (nx, ny) not in blocked:                # ② 不是墙
                result.append((nx, ny))
    return result


def reconstruct_path(came_from, start, goal):
    """从终点顺着 came_from 往回找到起点，返回从起点到终点的路线（格子列表）。"""
    path = [goal]
    cell = goal
    while cell != start:          # 还没回到起点，就继续往回找
        cell = came_from[cell]               # 这个格子是从哪来的？
        path.append(cell)
    path.reverse()                # 现在是“终点→起点”，反过来变成“起点→终点”
    return path


def heuristic(cell, goal):
    """估计从 cell 到 goal 还要走几步：横着差几格 + 竖着差几格。"""
    return abs(goal[0] - cell[0]) + abs(goal[1] - cell[1])


def astar(start, goal, blocked, width, height):
    """在格子地图上从 start 找到 goal，返回路线（格子列表）；找不到返回 None。"""
    # 第 1 块：三本账的初始值
    open_set = [start]
    g = {start: 0}
    came_from = {}

    while len(open_set) > 0:      # 还有待考察的格子，就继续找
        # 第 2 块：挑 f 最小的格子（下一步写）
        current = open_set[0]
        for cell in open_set:
            f_cell = g[cell] + heuristic(cell, goal)
            f_current = g[current] + heuristic(current, goal)
            if f_cell < f_current:
                current = cell

        open_set.remove(current)
        if current == goal:
            return reconstruct_path(came_from, start, goal)


        # 第 3 块：看邻居、贴纸条
        for nb in neighbors(current, blocked, width, height):
            new_g = g[current] + 1
            if nb not in g or new_g < g[nb]:      # 第一次见到，或者这次更近
                g[nb] = new_g                      # 更新步数
                came_from[nb] = current            # 贴纸条：nb 是从哪来的
                if nb not in open_set:
                    open_set.append(nb)           # 放进待考察

    return None                   # 所有能去的格子都看完了还没到终点：说明路被堵死了
