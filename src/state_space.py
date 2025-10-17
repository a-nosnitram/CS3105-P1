import code
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

    def get_neighbors(self, node: Node):
        # take obstacles into account
        if node is None:
            raise ValueError(
                "get_neighbors called with node=None; pass a Node instance")

        neighbors = []

        for o in self.obs:
            if node.coord in o.area and not node.coord in o.verts:
                return neighbors

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

                # edge case: when the plygon is small (perimeter == area)
                # lines that pass through the polygon edges need to be blocked too
                # we need to check if the line intersects a polygon edge that does not contain either endpoint
                # also, t might intersect the polygon in between coords
                for e in polygon.edges:
                    # exclude line edges that contain vertices with e.line[1] and e.line[len(e.line)-2]
                    if self.two_segments_intersect(node.coord, target_coord, e.line[0], e.line[len(e.line) - 1]):
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

    # check if two segments intersect, excluding endpoints
    def two_segments_intersect(self, p1, p2, q1, q2):
        # Check if line segments p1p2 and q1q2 intersect
        def orientation(p, q, r):
            val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)
            if val == 0:
                return 0  # collinear
            return 1 if val > 0 else 2  # clock or counterclock wise

        o1 = orientation(p1, p2, q1)
        o2 = orientation(p1, p2, q2)
        o3 = orientation(q1, q2, p1)
        o4 = orientation(q1, q2, p2)

        if o1 == 0 or o2 == 0 or o3 == 0 or o4 == 0:
            # This covers collinear overlap and touching at endpoints; return False per requirement.
            return False

        # General case
        if o1 != o2 and o3 != o4:
            return True

        return False
