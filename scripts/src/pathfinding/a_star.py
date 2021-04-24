import math

from collections import defaultdict
from functools import partial

from scripts.src.pathfinding.pathfinding_algorithm import PathfindingAlgorithm
from scripts.src.pathfinding.path_not_found_exception import PathNotFoundException
from scripts.src.pathfinding.tile_role import TileRole
from scripts.src.pathfinding.config import NODE_SIZE


class AStar(PathfindingAlgorithm):
    """https://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode"""
    def find_path(self, start, end):
        heuristic = partial(distance, point2=end.pixel_coordinates_center)
        openSet = {start}
        cameFrom = {}
        gScore = defaultdict(lambda: float('inf'))
        gScore[start] = 0

        fScore = defaultdict(lambda: float('inf'))
        fScore[start] = heuristic(start.pixel_coordinates_center)

        while openSet:
            openSet_fScores = {
                node: fScore[node] for node in openSet
            }

            current = min(openSet_fScores.items(), key=lambda x: x[1])[0]

            if distance(current.pixel_coordinates_center,
                        end.pixel_coordinates_center) \
                    < NODE_SIZE:
                return reconstruct_path(cameFrom, current)

            openSet.remove(current)
            for neighbor, _ in current.neighbors:

                if neighbor.role is TileRole.EMPTY or \
                        neighbor.role is TileRole.END or \
                        neighbor.role is TileRole.START:
                    tentative_gScore = gScore[current] + 1
                else:
                    tentative_gScore = float('inf')

                if tentative_gScore < gScore[neighbor]:
                    cameFrom[neighbor] = current
                    gScore[neighbor] = tentative_gScore
                    fScore[neighbor] = gScore[neighbor] + \
                                       heuristic(neighbor.pixel_coordinates_center)
                    if neighbor not in openSet:
                        openSet.add(neighbor)

        raise PathNotFoundException()


def distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt(pow(x2-x1, 2) + pow(y2-y1, 2))


def reconstruct_path(cameFrom, current):
    total_path = [current]
    while current in cameFrom.keys():
        current = cameFrom[current]
        total_path.append(current)
    return total_path[::-1]
