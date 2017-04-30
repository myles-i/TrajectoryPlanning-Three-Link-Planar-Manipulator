import matplotlib.pyplot as plt
import numpy as np
from math import *
from subprocess import call

# Trajectory Parameters

# ##Trajectory 1
# n     = 10 # number of points
# slew_time = 2
# start = [3,3,0] #[x,y,alpha]
# end   = [5,5,0] #[x,y,alpha]

# Trajectory 2
n     = 5 # number of points
slew_time = 2
start = [5,6,0] #[x,y,alpha]
end   = [-2,4,-pi/2] #[x,y,alpha]

##Trajectory  out of bounds
# n     = 10 # number of points
# slew_time = 2
# start = [3,3,0] #[x,y,alpha]
# end   = [20,5,0] #[x,y,alpha]

# Write to input file
Fw = open("input.txt","w")
Fw.writelines('n=' + str(n) + '\n')
Fw.writelines('slew_time=' + str(slew_time) + '\n')
Fw.writelines('start=' + str(start) + '\n')
Fw.writelines('end=' + str(end) + '\n')
Fw.close()

# Generate trajectory using compiled C file executable
call(["./Calc_Trajectory", "input.txt","output.txt"])

# Open and read outputs
F = open("output.txt","r")
print(F.readline())
# Get trajectory parameters
exec(F.readline())
exec(F.readline())
exec(F.readline())
exec(F.readline())
F.readline(); #blank line

plt.figure()
plt.plot([start[0],end[0]],[start[1],end[1]],'red')


J0 = np.zeros([2,1])
J1 = np.zeros([2,1])
J2 = np.zeros([2,1])
J3 = np.zeros([2,1])
L1 = 5.0
L2 = 4.0
L3 = 3.0

#Plot linkages through time
for i in range(n):
	exec('state = ' + F.readline())

	J3[0] = L1*cos(state[0]) + L2*cos(state[0] + state[1]) + L3*cos(state[0] + state[1] + state[2]);
	J3[1] = L1*sin(state[0]) + L2*sin(state[0] + state[1]) + L3*sin(state[0] + state[1] + state[2]);

	J2[0] = L1*cos(state[0]) + L2*cos(state[0] + state[1]);
	J2[1] = L1*sin(state[0]) + L2*sin(state[0] + state[1]);

	J1[0] = L1*cos(state[0]);
	J1[1] = L1*sin(state[0]);

	plt.plot([J0[0],J1[0],J2[0],J3[0]], [J0[1],J1[1],J2[1],J3[1]],'blue')

plt.title('3-Linkage Planar Manipulator Trajectory')
plt.legend(['Intended Trajectory in Cartesian','Linkages through time'])
plt.xlabel('x-axis')
plt.ylabel('y-axis')	
plt.show()


