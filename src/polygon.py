#********************CS3105 P1 2025-26 Starter Code
#
# This class stores a polygon as a sequence of vertexes assumed to be ordered in a clock-wise direction, such that connecting consecutive vertices form the edges of the polygon
# 
#  @author a.toniolo
# 
#


class Polygon:
	def __init__(self, verts:list):
		self.verts = verts
		self.edges = []
		self.perim = set()
		self.area = set()
	
	def setVertexes(self, v:list):
		self.verts = v

	def setEdges(self, e:list):
		self.edges = e

	def setArea(self, a:set):
		self.area = a

	def setPerimeter(self, p:set):
		self.perim = p
	
	