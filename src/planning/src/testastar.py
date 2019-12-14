from astar import *

map = [
[0, 1, 1, 0, 0, 1],
[0, 0, 1, 1, 0, 0],
[0, 0, 0, 0, 1, 0],
[1, 0, 0, 1, 1, 0],
[1, 1, 0, 1, 0, 0],
[0, 0, 0, 0, 0, 0]]
start = [0, 0]
goal = [5, 4]
search = Search(map, start, goal)
path = search.astar()
print(path)
path1 = search.findPath()
print(path1)
