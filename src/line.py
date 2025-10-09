#********************CS3105 P1 2025-26 Starter Code
#
# This class stores a line as a sequence of coordinates and its length
# It contains a simple method to calculate the length although the digitiser already sets the length. 
# 
#
#@author a.toniolo
#
#
import math

def measure(line):
# Assume the coordinates are in order
	l=0 
	c1=line[0]
	for i in range(1,len(line)):
		c2=line[i]
		if (c1.x==c2.x or c1.y==c2.y):
			l=l+1
		else:
			l=l+math.sqrt(2)
		c1=c2
	return l


class Line:
    def __init__(self,*args):
        if(len(args)==2):
            self.line=args[0]
            self.length=args[1]
        else:
            self.line=args[0]
            self.length=measure(args[0])
 

