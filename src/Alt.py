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

        result_cost, nodes_cnt, _ = self.recursive_helper(start_node, float('inf'), 0)

        # If no path found and verbose, print node count
        if result_cost is None and self.verbose:
            print(nodes_cnt)

        return result_cost, nodes_cnt

    def recursive_helper(self, node, f_limit, nodes_explored):
        # first base case: goal reached
        nodes_explored += 1

        if node.coord == self.state_space.goal:
            if self.verbose:
                print(nodes_explored)
            return node.cost, nodes_explored, node.total_cost

        # Get neighbors
        neighbors = self.state_space.get_neighbors(node)
        if not neighbors:
            return None, nodes_explored, float('inf')  # fail, return infinite f-value

        # find successors from neighbours (with cycle detection)
        successors = self.find_successors(neighbors, node)

        if not successors:
            return None, nodes_explored, float('inf')

        while True:
            if self.verbose:
                print(str([str(n.coord) + "{:.1f}".format(n.total_cost)
                      for n in successors]).replace("'", ""))

            # Get best and second-best f-values
            best = successors[0]
            f_best = best.total_cost

            # if best successor has infinite cost, no path exists
            if f_best == float('inf'):
                return None, nodes_explored, float('inf')

            if f_best > f_limit:
                return None, nodes_explored, f_best  # return the best f-value we found

            # Get second-best f-value
            f_second = successors[1].total_cost if len(successors) > 1 else float('inf')

            # Recurse with new f-limit = min(f_limit, f_second)
            result_cost, node_cnt, new_f = self.recursive_helper(best, min(f_limit, f_second), nodes_explored)

            if result_cost is not None:
                return result_cost, node_cnt, new_f

            # Update the f-value of the failed path and re-sort
            best.total_cost = max(new_f, f_best)  # Use max to ensure monotonicity
            successors.sort(key=lambda x: x.total_cost)
            nodes_explored = node_cnt

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
            # Only add if not in current path (check parent chain for cycles)
            if not self.in_path(neighbor, node):
                successors.append(neighbor_node)

        # total_cost = f_value
        successors.sort(key=lambda x: x.total_cost)
        return successors

    def in_path(self, coord, node):
        """Check if coord is in the path from start to current node"""
        current = node
        while current is not None:
            if current.coord == coord:
                return True
            current = current.parent
        return False