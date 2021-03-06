import os
import math
from os import walk, getcwd


outpath = "/home/mma2739/lockheed_ws/src/control_node/waypoints/"
name = "start2.wp"
txt_outpath = outpath + name

txt_outfile = open(txt_outpath, "w")

print txt_outfile
# gen circle points
# plane is at 0, 10
x0 = 0
y0 = 10
alt = 3.5

for z in range(2,3,1):
	alt = 2.5 + z
	print 'new z'
	for i in range(0,2,1):
		r = i + 6 
		print 'new r'
		for theta in range(-90, 270, 15):
			x = r*math.cos(math.radians(theta)) + x0
			y = r*math.sin(math.radians(theta)) + y0
			psi = theta + 90
			txt_outfile.write(str(x) + " " + str(y) + " " + str(alt) + " " + str(psi) + "\n")