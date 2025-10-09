# ********************CS3105 P1 2025-26 Starter Code
#
# This is the main class
# It processes the input
# Then creates the obstacles
# Then calls the search algorithm
#
# @author a.toniolo
#
#


import sys
from coord import Coord
from polygon import Polygon
import digitiser
import copy


def printMap(start: Coord, goal: Coord, size: int, obs: set, add: set):
    # map legend:
    # '.': free point,
    # '0': obstacle
    # 'S': start
    # 'G': goal
    # '*': coordinates contained in 'add'
    #
    block = set()
    # collect the obstacles
    for o in obs:
        block.update(o.area)
    print()
    # print map
    print('  ', end='')
    for x in range(0, size):
        print(x % 10, end=" ")
    print()
    for y in range(0, size):
        print(y % 10, end=" ")
        for x in range(0, size):
            ch = '.'
            test = Coord(x, y)
            # decide what element to print on this cell
            if (test in block):
                ch = 'O'
            if (test in add):
                ch = '*'
            if (test == start):
                ch = 'S'
            if (test == goal):
                ch = 'G'
            print(ch, end=' ')
        print()
    print()


def runSearch(probType: str, algo: str, size: int, start: Coord, goal: Coord, obs: set,  verbose: bool):
    # here you are required to implement the search algorithms
    # make sure you are running the correct one depending of the following flags:
    # algo can be "BestF", "AStar", or "Alt"
    # probType can be "Default", "Two", or "Lines"
    # if you attempt part C, only one of "Alt", "Two", or "Lines" should be chosen
    # please make sure that for all the options not implemented you return the value -1.0=NOT_IMPLEMENTED
    # return NOT_IMPLEMENTED

    if algo == "AStar":
        from AStar import AStar
        return AStar(size, start, goal, obs, verbose).search()
    elif algo == "BestF":
        from BestF import BestF
        return BestF(size, start, goal, obs, verbose).search()


NOT_IMPLEMENTED = -1.0
NOT_FOUND = 0.0


if (len(sys.argv) < 2):
    print(
        "usage: python3 P1main.py <AStar|BestF|Alt> <prob>  [<Two|Lines>] [<verbose>]")
    sys.exit()

# *************** Process Input ***************#

# assume problem is correctly specified, set the problem components
algo = sys.argv[1]
probFile = "../CS3105-Tests/PROBS/"+sys.argv[2]+".txt"
scan = open(probFile)
scanLines = scan.readlines()
verbose = False
if ('verbose' in sys.argv):
    verbose = True
probType = "Default"  # basic problem
if ('Two' in sys.argv):
    probType = "Two"
if ('Lines' in sys.argv):
    probType = "Lines"

# first line:size of square surface
size = int(scanLines[0])

# second line:start coordinates
startCoord = scanLines[1]
start = Coord(startCoord)

# third line:goal coordinates
goalCoord = scanLines[2]
goal = Coord(goalCoord)

# fourth onwards: obstacles = list of coordinates indicating vertexes of polygons.
# it is assumed that vertexes are ordered in a clock-wise direction, such that connecting consecutive vertices form the edges of the polygon
obs = []
for count in range(3, len(scanLines)):
    next_line = scanLines[count]
    split = next_line.split(')(')
    vxs = []
    for o in split:
        c = Coord(o)
        vxs.append(c)
    poly = digitiser.drawPolygon(vxs)
    obs.append(poly)

# *************** Print settings ***************#
print("Algorithm: "+algo)
print("Prob Type: "+probType)
add = set()
printMap(start, goal, size, obs, add)
# this method can be used for debugging by including in 'add' all coordinates of the lines forming a path

# *************** Run and Output ***************#

# run the search
pathCost = runSearch(probType, algo, size, start, goal, obs,  verbose)

# remember to include required printing for verbose mode
# Eg
# Frontier
# [(1,0)5.0]
# [(3,1)2.8, (1,3)4.0]
# [(5,3)0.0, (1,3)4.0, (4,5)2.2]
# Explored nodes
# 3
# path vertexes
# (1,0)(3,1)(5,3)
#

# print the path cost, double value rounded to one decimal place
# if pathCost not found, print 0.0
if pathCost is None:
    print(0.0)
else:
    print("{:.1f}".format(pathCost))
