from coord import Coord
from node import Node
from digitiser import DrawLine
# edges of polygons are listed as neighbors in obs
# the agent can only move the following ways:
# - along a line that connects two vertices of a polygon
# - a line that connects start to goal
# - a line that connects start to a vertex of a polygon
# - a line that connects the vertex of one polygon to the vertex of another polygon


class StateSpace:
    def __init__(self, size: int, start: Coord, goal: Coord, obs: set):
        self.size = size
        self.start = start
        self.goal = goal
        self.obs = obs
        self.nodes = set()
        self.create_nodes()

    def create_nodes(self):
        self.nodes.add(self.start)
        self.nodes.add(self.goal)
        for o in self.obs:
            for v in o.verts:
                self.nodes.add(v)

    def can_go_to_goal(self, start: Coord, goal: Coord):
        line = DrawLine(start, goal)
        # check if line intersects any obstacle edge
        for polygon in self.obs:
            for point in line.line:
                if point in polygon.area:
                    return False
        return True

    def get_neighbors(self, node: Node):
        # take obstacles into account
        if node is None:
            raise ValueError(
                "get_neighbors called with node=None; pass a Node instance")

        neighbors = []

        # check all nodes in the visibility graph
        for target_coord in self.nodes:
            if target_coord == node.coord:
                continue  # skip self

            # line to target
            line = DrawLine(node.coord, target_coord)
            blocked = False

            # check if line passes through any obstacle
            for polygon in self.obs:
                # if both endpoints are vertices of THIS polygon
                # they might share an edge which is fine
                both_are_vertices = (node.coord in polygon.verts and
                                     target_coord in polygon.verts)

                if both_are_vertices:
                    # check if they are adjacent vertices (direct edge)
                    verts = polygon.verts
                    is_edge = False
                    for i in range(len(verts)):
                        next_i = (i + 1) % len(verts)
                        if ((verts[i] == node.coord and verts[next_i] == target_coord) or
                                (verts[i] == target_coord and verts[next_i] == node.coord)):
                            is_edge = True
                            break

                    if is_edge:
                        # This is a valid polygon edge; skip blocking check for this polygon
                        continue
                    else:
                        # Both are vertices but NOT adjacent
                        blocked = True
                        break

                # check if line intersects polygon area (interior or perimeter)
                for point in line.line:
                    # skip vertices
                    if point == node.coord or point == target_coord:
                        continue
                    # block if point touches polygon area at all
                    if point in polygon.area:
                        blocked = True
                        break

                if blocked:
                    break

            if not blocked:
                neighbors.append(target_coord)

        return neighbors
