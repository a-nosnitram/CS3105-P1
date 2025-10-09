#********************CS3105 P1 2025-26 Starter Code
# 
# This class constructs a coordinate data structure (x,y)
# It can read both a string "(x,y)" or a couple of integers x and y
# 
#  
# @author a.toniolo
# 




class Coord:
	def __init__(self,*args):
		if(len(args)==2):
			self.x = int(args[0])
			self.y = int(args[1])
		elif(len(args)==1):
			coord=args[0].replace(' ','')
			coord=coord.replace('(','')
			coord=coord.replace(')', '')
			split=coord.split(',')
			self.x = int(split[0])
			self.y = int(split[1])
		else:
			self.x=0
			self.y=0
	
	def __str__(self):
		return  '('+str(self.x)+','+str(self.y)+')'
	
	def __repr__(self):
		return  '('+str(self.x)+','+str(self.y)+')'

	def sort_by_x(self):
		return self.x
	
	def __eq__(self,a):
		return str(self)==str(a)
	
	def __hash__(self):
   		return hash(str(self)) 
	
	
		