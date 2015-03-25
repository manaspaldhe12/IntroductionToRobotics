This file contains the running details of the project1_ManasPaldhe.py file

REQUIREMENTS:
Python 2.7
Openrave 0.6 or higher
The project requires input.file to be in the same folder as the project1_ManasPaldhe.py file

INPUT:
The input file should consist of exactly six rows.
Each row is of the format
theta(i), d(i), alpha(i), a(i)

theta and alpha must be in degrees
d and a must be in centmeter

OUTPUT:
After succesful completion a file output.file is created.
For each angle the forward and inverse kinematics are computed and results are stored in this file.

Kindly note that some Geometric solutions may 'seem' incorrect for theta(4), theta(5) and theta(6).
This is because the FLIP is so chosen

CONFIGURATION:
Rows 77 to 80 of the python file are configuration parameters.
Set them as required.

