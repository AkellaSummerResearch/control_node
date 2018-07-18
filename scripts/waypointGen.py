import os
import math
from os import walk, getcwd


outpath = "/home/lockheed/catkin_ws/src/control_node/waypoints/"
name = "train.wp"
txt_outpath = outpath + name

txt_outfile = open(txt_outpath, "w")

print txt_outfile
# gen circle points
# plane is at 0, 10
x0 = 0
y0 = 10
alt = 2

for i in range(0,4,1):
	r = i*.5 + 7

	for theta in range(270, 630, 5):
		x = r*math.cos(math.radians(theta)) + x0
		y = r*math.sin(math.radians(theta)) + y0
		psi = -theta - 90
		txt_outfile.write(str(x) + " " + str(y) + " " + str(alt) + " " + str(psi) + "\n")