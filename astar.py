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
