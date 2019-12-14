from astar import *

# map = [
# [0, 1, 1, 0, 0, 1],
# [0, 0, 1, 1, 0, 0],
# [0, 0, 0, 0, 1, 0],
# [1, 0, 0, 1, 1, 0],
# [1, 1, 0, 1, 0, 0],
# [0, 0, 0, 0, 0, 0]]
map = [[1, 1, 1, 1, 1, 1],
       [1, 0, 0, 0, 0, 1],
       [1, 0, 0, 1, 0, 0],
       [0, 0, 0, 1, 0, 0],
       [1, 1, 1, 1, 1, 1]]

start = (3, 2)
goal = (3, 5)
planner = Planner(map, start, goal)
path = planner.astar()
print(path)
path1 = planner.findPath()
print(path1)
