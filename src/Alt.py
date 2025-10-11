# Recursive Best First Search Algorithm
# this one is similar to recursive depth-first search
# but it uses an evaulation limit to keep track of
# the best alternative path available from any ancestor node
# if the current path exceeds this limit, it backtracks

from queue import PriorityQueue
from node import Node
from state_space import StateSpace
import itertools
from digitiser import DrawLine

class Alt():
    def __init__(self, size, start, goal, obs, verbose):
        self.state_space = StateSpace(size, start, goal, obs)
        self.verbose = verbose
        self.frontier = PriorityQueue()
        self._counter = itertools.count()
        self.alt = None # best alternative path cost

    def heuristic(self, coord):
        # euclidean distance
        return ((coord.x - self.state_space.goal.x) ** 2 + (coord.y - self.state_space.goal.y) ** 2) ** 0.5

    def search(self):
        start_node = Node(self.state_space.start)
        start_node.cost = 0
        start_node.her = self.heuristic(self.state_space.start)
        start_node.total_cost = start_node.cost + start_node.her

        self.frontier.put(
            (start_node.total_cost, next(self._counter), start_node))

        explored = set()

        return self.recursive_helper(start_node, float('inf'), explored)

    def recursive_helper(self, node, f_limit, explored):
        # first base case: goal reached
        if node.coord == self.state_space.goal:
            if self.verbose:
                    print(len(explored))
                    print("".join([str(coord)
                          for coord in explored]).replace(" ", ""))
            return node.cost, len(explored)

        # if node in explored, continue
        coord_key = (node.coord.x, node.coord.y)
        if coord_key in explored:
            if self.verbose:
                    print(len(explored))
            return None, len(explored)

        # add node to explored
        explored.add(coord_key)

        # Get neighbors
        neighbors = self.state_space.get_neighbors(node)
        if not neighbors:
            explored.remove(coord_key)
            if self.verbose:
                    print(len(explored))
            return None, len(explored)  # fail

        # find successors from neighbours
        successors = self.find_successors(neighbors, node)

        while successors:
            f_value, best = successors[0].total_cost, successors[0]

            if f_value > f_limit:
                explored.remove(coord_key)
                if self.verbose:
                    print(len(explored))
                return None, len(explored) # fail

            second_best = successors[1].total_cost if len(successors) > 1 else float('inf')

            result_cost, node_cnt = self.recursive_helper(best, min(f_limit, second_best), explored)

            if result_cost is not None:
                explored.remove(coord_key)
                if self.verbose:
                    print(len(explored))
                    print("".join([str(coord)
                          for coord in explored]).replace(" ", ""))
                return result_cost, node_cnt

            # if failed, mark as nfinite cost ad try next successor
            successors[0].total_cost = float('inf')
            successors.sort(key=lambda x: x.total_cost)

        explored.remove(coord_key)
        if self.verbose:
            print(len(explored))
        return None, len(explored)

    def calculate_f_value(self, current_node, neighbor):
        line = DrawLine(current_node.coord, neighbor.coord)
        distance = line.length
        neighbor.cost = current_node.cost + distance
        neighbor.her = self.heuristic(neighbor.coord)
        neighbor.total_cost = neighbor.cost + neighbor.her
        return neighbor.total_cost

    def find_successors(self, neighbors, node):
        successors = []
        for neighbor in neighbors:
            neighbor_node = Node(neighbor, node)
            self.calculate_f_value(node, neighbor_node)
            successors.append(neighbor_node)

        # total_cost = f_value
        successors.sort(key=lambda x: x.total_cost)
        return successors