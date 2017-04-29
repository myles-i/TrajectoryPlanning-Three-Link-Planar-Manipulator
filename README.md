# TrajectoryPlanning-Three-Link-Planar-Manipulator

## Building Project
The project can be built using the gcc compiler as follows:

gcc -o Calc_Trajectory Calc_Trajectory.c

## Running the trajectory generator
To calculate a trajectory:
1) Specify the number of points, start position [x,y,phi], and end position [x,y,phi] in an input file with the following format:

n=10
start=[3,3,0]
end=[5,5,0]

2) Call the executable with the input file and output file name as the first and second arguments. The output trajectory will be written to the output file:

./Calc_Trajectory input.txt output.txt