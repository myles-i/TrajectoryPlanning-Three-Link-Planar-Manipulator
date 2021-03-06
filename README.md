# TrajectoryPlanning-Three-Link-Planar-Manipulator
This project computes trajectories for a three linkage, planar robot given a starting cartesian position and manipulator orientation [x,y,alpha] and an ending position/orientation. The computed trajectory moves the manipulator from the starting to ending position at constant speed (in x,y and alpha), and provides each of the link angles and speeds a "n" (n is specified) points throughout the trajectory. Outputs are printed to an output file, and to the screen in the following format:
[Theta1, Theta2, Theta3, dTheta1/dt, dTheta2/dt, dTheta3/dt]

## Building Project
The project can be built using the gcc compiler as follows:

gcc -o Calc_Trajectory Calc_Trajectory.c

## Running the trajectory generator
To calculate a trajectory:
1) Specify the number of points, slew_time (seconds), start position [x,y,phi], and end position [x,y,phi] in an input file with the following format:

n=10
slew_time=2
start=[3,3,0]
end=[5,5,0]

2) Call the executable with the input file and output file name as the first and second arguments. The output trajectory will be written to the output file:

./Calc_Trajectory input.txt output.txt

## Running the trajectory generator with plotting
You can also run and plot c file and plot the results using the following python command.  assumes the executable has already been created with name "Calc_Trajectory". The python file can be edited to plot different trajectories:

python plot_trajectory.py

Python dependencies:
matplotlib
numpy
