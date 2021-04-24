"""
Algorithm that finds a path from a starting node to the first node met with a TileRole.END role.
"""

from collections import deque

from pathfinding.pathfinding_algorithm import PathfindingAlgorithm
from pathfinding.path_not_found_exception import PathNotFoundException
from pathfinding.tile_role import TileRole


class BreadthFirstSearch(PathfindingAlgorithm):
    """
    Algorithm that finds a path from a starting node to the first node met with a TileRole.END role.
    """
    def find_path(self, start, _end):
        queue = deque()
        visited = {start}

        queue.append([start])

        while queue:
            path = queue.popleft()
            node = path[-1]

            if node is _end:
                return path

            for neighbor, _ in node.neighbors:
                if neighbor not in visited and (neighbor.role is TileRole.EMPTY
                                                or neighbor.role is TileRole.END
                                                or neighbor.role is TileRole.START
                                                ):
                    queue.append(path + [neighbor])
                    visited.add(neighbor)

        raise PathNotFoundException()

    def find_path_to_empty_space(self, start):
        queue = deque()
        visited = {start}

        queue.append([start])

        while queue:
            path = queue.popleft()
            node = path[-1]

            if node.role is TileRole.EMPTY:
                return path

            for neighbor, _ in node.neighbors:
                queue.append(path + [neighbor])
                visited.add(neighbor)

        raise Exception("find path to empty space exception, what the actual fuck")
