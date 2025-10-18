# A* Search Algorithm

from queue import PriorityQueue
from node import Node
from state_space import StateSpace
from coord import Coord
import itertools
from digitiser import DrawLine


class AStar():
    def __init__(self, size, start, goal, obs, verbose):
        self.state_space = StateSpace(size, start, goal, obs)
        self.verbose = verbose
        self.frontier = PriorityQueue()
        self._counter = itertools.count()

    def heuristic(self, coord):
        # euclidean distance
        return ((coord.x - self.state_space.goal.x) ** 2 + (coord.y - self.state_space.goal.y) ** 2) ** 0.5

    # search through state space, where you're only allowed to move
    # along obstacle edges (not through obstacles)
    def search(self):
        start_node = Node(self.state_space.start)
        start_node.cost = 0
        start_node.her = self.heuristic(self.state_space.start)
        start_node.total_cost = start_node.cost + start_node.her

        self.frontier.put(
            (start_node.total_cost, next(self._counter), start_node))

        explored = set()
        step = 0
        while not self.frontier.empty():
            if self.verbose:
                print(str([str(n.coord) + "{:.1f}".format(n.total_cost)
                      for _, _, n in self.frontier.queue]).replace("'", ""))

            # queue items are (total_cost, counter, node)
            current_node = self.frontier.get()[2]
            explored.add((current_node.coord.x, current_node.coord.y))
            if current_node.coord == self.state_space.goal:
                if self.verbose:
                    print(len(explored))
                    print("".join([str(coord)
                          for coord in explored]).replace(" ", ""))
                return float(current_node.cost), len(explored)
            # get neighbors
            neighbors = self.state_space.get_neighbors(current_node)
            for neighbor in neighbors:
                neighbor_node = Node(neighbor, current_node)

                line = DrawLine(current_node.coord, neighbor)
                distance = line.length
                neighbor_node.cost = current_node.cost + distance
                neighbor_node.her = self.heuristic(neighbor)
                neighbor_node.total_cost = neighbor_node.cost + neighbor_node.her

                coord_key = (neighbor_node.coord.x, neighbor_node.coord.y)
                if coord_key not in explored:
                    in_frontier = False
                    for item in self.frontier.queue:
                        if item[2].coord == neighbor_node.coord and item[2].total_cost <= neighbor_node.total_cost:
                            in_frontier = True
                            break
                    if not in_frontier:
                        self.frontier.put(
                            (neighbor_node.total_cost, next(self._counter), neighbor_node))
            step += 1
        if self.verbose:
            print(len(explored))
        return None, len(explored)  # no path found :(
