# ********************CS3105 P1 2025-26 Starter Code
#
# This class is used to construct the components of a polygon, from its vertexes
# First it draws the edges (stored as Lines) using the Bresenham's algorithm.
# The perimeter is stored as a sequence of coordinates
# Then it fills the area using an adapted ScanLine algorithm
#
# https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
# https://www.cs.rit.edu/~icss571/filling/how_to.html
#
# @author a.toniolo
#
#


from coord import Coord
from line import Line
from polygon import Polygon
import math
import sys

# This method implements the Bresenham's algorithm.
# This determines the points of a 2D map to form a close approximation to
# a straight line between two given points.
# The code uses the pseudocode presented in the Wikipedia's page, adapted for Java
# https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
#


def DrawLine(a: Coord, b: Coord):
    x0 = a.x
    y0 = a.y
    x1 = b.x
    y1 = b.y

    if (abs(y1 - y0) < abs(x1 - x0)):
        if (x0 > x1):
            return plotLineLow(x1, y1, x0, y0)
        else:
            return plotLineLow(x0, y0, x1, y1)
    else:
        if (y0 > y1):
            return plotLineHigh(x1, y1, x0, y0)
        else:
            return plotLineHigh(x0, y0, x1, y1)


#
# Bresenham's algorithm Helper method
# https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
#

def plotLineLow(x0: int, y0: int, x1: int, y1: int):
    line = []
    c = 0
    Dc = 0
    dx = x1 - x0
    dy = y1 - y0
    yi = 1

    if (dy < 0):
        yi = -1
        dy = -dy
    D = (2 * dy) - dx
    y = y0
    for x in range(x0, x1+1):
        line.append(Coord(x, y))
        c = c+Dc
        if (D > 0):
            y = y + yi
            D = D + (2 * (dy - dx))
            # goes diagonal
            Dc = math.sqrt(2)
        else:
            D = D + 2*dy
            # only x changes
            Dc = 1
    lx = Line(line, c)
    return lx


#
# Bresenham's algorithm Helper method
# https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
#

def plotLineHigh(x0: int, y0: int, x1: int, y1: int):
    line = []
    c = 0
    Dc = 0
    dx = x1 - x0
    dy = y1 - y0
    xi = 1
    if (dx < 0):
        xi = -1
        dx = -dx
    D = (2 * dx) - dy
    x = x0
    for y in range(y0, y1+1):
        line.append(Coord(x, y))
        c = c+Dc
        if (D > 0):
            x = x + xi
            D = D + (2 * (dx - dy))
            # goes diagonal
            Dc = math.sqrt(2)
        else:
            D = D + 2*dx
            # only y changes
            Dc = 1
    lx = Line(line, c)
    return lx


# This method draws the approximate polygon given by the vertexes.
# For each pair of vertexes draw a line,
# then set the correspondent elements of the polygon

def drawPolygon(verts: list):

    poly = Polygon(verts)
    perim = set()
    edges = []
    perim.update(verts)
    first = verts[0]
    for i in range(1, len(verts)):
        second = verts[i]
        l = DrawLine(first, second)
        edges.append(l)
        perim.update(l.line)
        first = second
    l = DrawLine(first, verts[0])
    edges.append(l)
    perim.update(l.line)
    poly.setArea(fillPolygon(verts, perim))
    poly.setEdges(edges)
    poly.setPerimeter(perim)
    return poly

# ScanLine's algorithm  adapted to convex polygons drawn with Bresenham's algorithm
# https://www.cs.rit.edu/~icss571/filling/how_to.html


def fillPolygon(verts: list, edges: list):
    fill = set(edges)
    map = {}

    ymin = sys.maxsize
    ymax = 0

    # Step 1: take y max and min to determine the area to scan
    for f in fill:
        if (f.y < ymin):
            ymin = f.y
        if (f.y > ymax):
            ymax = f.y
        buck = set()
        if (f.y in map.keys()):
            buck = map.get(f.y)
        buck.add(f)
        map.update({f.y: buck})

    # Step 2: scan each line from min to max y and fill the line between two edges intersecting it

    for scan in range(ymin, ymax+1):
        # for each line, find the edges that intersect line
        cross = map.get(scan)
        cx = list(cross)
        gap = False
        x1 = None
        x2 = None

        # order them by x
        qx = sorted(cx, key=Coord.sort_by_x)

        # find a gap between them if it exists
        for i in range(0, len(qx)-1):
            x1 = qx[i]
            x2 = qx[i+1]
            if ((x1.x+1) != x2.x):
                gap = True
                break
        if (gap):
            # fill the gap between x1 and x2
            for i in range(x1.x, x2.x+1):
                fill.add(Coord(i, scan))

    # this is now the area
    return fill
