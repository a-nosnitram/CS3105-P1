from coord import Coord


class Node:
    def __init__(self, coord: Coord, parent=None):
        self.coord = coord
        self.parent = parent
        self.cost = 0  # cost from start to this node
        self.her = 0  # heuristic cost to goal
        self.total_cost = 0  # total cost

    def __eq__(self, other):
        return self.coord == other.coord
