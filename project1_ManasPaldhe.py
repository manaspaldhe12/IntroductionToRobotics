#########################################################################################
# ECE 569 - Introduction to Robotic Systems #
#               Project 1                   #
# Developed by- Manas Paldhe                #
#               0025908254                  #
#########################################################################################

#########################################################################################
## Includuing header files
from openravepy import *
import time
from numpy import *
from math import *

#########################################################################################
# The project #
#1) Write a python or C++ program to run in OpenRave to verify the direct and inverse kinemat-
#ics of the PUMA 560 robot shown in Figure 2.11 of your textbook. Use the parameters in the
#D-H table in Figure 2.11. The OpenRave program should be able to do all these functions:

#(a) Read the D-H parameters from a file (given by the user).

#(b) Output all its link transformation matrices, i-1 A i, in a symbolic form (as a check to see
#your input is correct).

#(c) Forward Kinematics checking: Input all the joint ranges of PUMA robot joint angles,
#your software will loop through all the joint angles at a preset incremental of joint angle
#(say 5 degrees or one degree); the software will also show the PUMA robot motion for
#each increment of joint angles.

#(d) Inverse Kinematics checking: For each data point of joint angles, obtain (i) robot con-
#figuration indices (i.e., ARM, ELBOW, and WRIST, etc.), and (ii) the end-effector
#location in terms of 4x4 HTM. Use the HTM of the end-effector and the robot con-
#figuration indices, find its joint angles. This set of joint angles should agree with your
#inputed joint angles in forward kinematics.


### Header functions ####

## Matrix Multiplication
def matrixmult (A, B):
    rows_A = len(A)
    cols_A = len(A[0])
    rows_B = len(B)
    cols_B = len(B[0])

    if cols_A != rows_B:
      print "Cannot multiply the two matrices. Incorrect dimensions."
      return

    # Create the result matrix
    # Dimensions would be rows_A x cols_B
    C = [[0 for row in range(cols_B)] for col in range(rows_A)]
    #print C

    for i in range(rows_A):
        for j in range(cols_B):
            for k in range(cols_A):
                C[i][j] += A[i][k]*B[k][j]
    return C

### End Matrix Multiplication

## PART A -> Read the D-H coordinates from the input.file file and store them in variables
file_read = open("input.file")
sum=0

theta=[10,30,43,20,30,04]
d=[0,10,0,10,0,0]
alpha=[-90,0,90,-90,90,0]
a=[0,10,5,0,0,0]

pos=[0,0,0]
count=0

## CONFIGURATION ##
min_angles=[-135, -135, -135, -135, 100, 100]
max_angles=[ 180,  180,  180,  180, 101, 101]
delta=45;
FLIP=-1
## 

for line_counter in range (1,7):
    line = file_read .readline()
    if not line:
	print "error in input file at line %d" % (line_counter)
        break
    length =len(line)
    pos=[0,0,0]
    count=0
    #print "%d" %(line_counter)
    for parser in range(0, length):
	if line[parser]==',':
		count=count+1
		pos[count-1]=parser
		#print "parser pos %d " %(pos[count-1])
    
    theta[line_counter-1]=float(line[0:pos[0]])
    d[line_counter-1]=float(line[pos[0]+1:pos[1]])
    alpha[line_counter-1]=float(line[pos[1]+1:pos[2]])
    a[line_counter-1]=float(line[pos[2]+1:])
    
    if count != 3:
	print "error in input file at line %d" %(line_counter)

file_read .close()

print theta
print d
print alpha
print a
print ""

file_write = open("output.file", "wb")
file_write.write( "Welcome\n  This project is developed by Manas Paldhe PUID: 0025908254 \n File read complete \n");

## End of PART A

## PART B -> Getting the A matrices in symbolic form
# (i-1)A(i)=
#		cos(theta)	-cos(alpha)sin(theta)	sin(alpha)sin(theta)	acos(theta)
#		sin(theta)	cos(alpha)cos(theta)	-sin(alpha)cos(theta)	asin(theta)
#		0		sin(alpha)		cos(alpha)		d
#		0		0			0			1

file_write.write("\n PRINTING THE POSITION MATRIX IN SYMBOLIC \n")
file_write.write( "A_01=	[	[float(cos(theta[0])),	-float(cos(alpha[0]))*float(sin(theta[0])),	float(sin(alpha[0]))*float(sin(theta[0])),	float(a[0])*float(cos(theta[0]))], \n")
file_write.write( "		[float(sin(theta[0])),	float(cos(alpha[0]))*float(cos(theta[0])),	-float(sin(alpha[0]))*float(cos(theta[0])),	float(a[0])*float(sin(theta[0]))], \n")
file_write.write( "		[0,		float(sin(alpha[0])),			float(cos(alpha[0])),			float(d[0])], \n")
file_write.write( "		[0,		0,				0,				1]		] \n")
file_write.write( "\n")

file_write.write( "A_12=	[	[float(cos(theta[1])),	-float(cos(alpha[1]))*float(sin(theta[1])),	float(sin(alpha[1]))*float(sin(theta[1])),	float(a[1])*float(cos(theta[1]))], \n") 
file_write.write( "		[float(sin(theta[1])),	float(cos(alpha[1]))*float(cos(theta[1])),	-float(sin(alpha[1]))*float(cos(theta[1])),	float(a[1])*float(sin(theta[1]))], \n")
file_write.write( "		[0,		float(sin(alpha[1])),			float(cos(alpha[1])),			float(d[1])], \n")
file_write.write( "		[0,		0,				0,				1]		] \n")
file_write.write( "\n")

file_write.write( "A_23=	[	[float(cos(theta[2])),	-float(cos(alpha[2]))*float(sin(theta[2])),	float(sin(alpha[2]))*float(sin(theta[2])),	float(a[2])*float(cos(theta[2]))], \n")
file_write.write( "		[float(sin(theta[2])),	float(cos(alpha[2]))*float(cos(theta[2])),	-float(sin(alpha[2]))*float(cos(theta[2])),	float(a[2])*float(sin(theta[2]))], \n")
file_write.write( "		[0,		float(sin(alpha[2])),			float(cos(alpha[2])),			float(d[2])], \n")
file_write.write( "		[0,		0,				0,				1]		] \n")
file_write.write( "\n")

file_write.write( "A_34=	[	[float(cos(theta[3])),	-float(cos(alpha[3]))*float(sin(theta[3])),	float(sin(alpha[3]))*float(sin(theta[3])),	float(a[3])*float(cos(theta[3]))], \n")
file_write.write( "		[float(sin(theta[3])),	float(cos(alpha[3]))*float(cos(theta[3])),	-float(sin(alpha[3]))*float(cos(theta[3])),	float(a[3])*float(sin(theta[3]))], \n")
file_write.write( "		[0,		float(sin(alpha[3])),			float(cos(alpha[3])),			float(d[3])], \n")
file_write.write( "		[0,		0,				0,				1]		] \n")
file_write.write( "\n")

file_write.write(  "A_45=	[	[float(cos(theta[4])),	-float(cos(alpha[4]))*float(sin(theta[4])),	float(sin(alpha[4]))*float(sin(theta[4])),	float(a[4])*float(cos(theta[4]))], \n")
file_write.write(  "		[float(sin(theta[4])),	float(cos(alpha[4]))*float(cos(theta[4])),	-float(sin(alpha[4]))*float(cos(theta[4])),	float(a[4])*float(sin(theta[4]))], \n")
file_write.write(  "		[0,		float(sin(alpha[4])),			float(cos(alpha[4])),			float(d[4])], \n")
file_write.write(  "		[0,		0,				0,				1]		] \n")
file_write.write( "\n")

file_write.write(  "A_56=	[	[float(cos(theta[5])),	-float(cos(alpha[5]))*float(sin(theta[5])),	float(sin(alpha[5]))*float(sin(theta[5])),	float(a[5])*float(cos(theta[5]))], \n")
file_write.write(  "		[float(sin(theta[5])),	float(cos(alpha[5]))*float(cos(theta[5])),	-float(sin(alpha[5]))*float(cos(theta[5])),	float(a[5])*float(sin(theta[5]))], \n")
file_write.write(  "		[0,		float(sin(alpha[5])),			float(cos(alpha[5])),			float(d[5])], \n")
file_write.write(  "		[0,		0,				0,				1]		] \n")
file_write.write( "\n")

## End of PART B

## PART C -> Run the program for joint angles
try:
	env = Environment()
	env.SetViewer('qtcoin')
	env.Load('data/puma_tabletop.env.xml') # load a simple scene
	robot = env.GetRobots()[0] # get the first robot
        robot.SetDOFValues(0*array([1,1,1,1,1,1,1]),[0,1,2,3,4,5,6])
	T0 = robot.GetLinks()[0].GetTransform() # get the transform of link 1
    	T1 = robot.GetLinks()[1].GetTransform() # get the transform of link 2
    	T2 = robot.GetLinks()[2].GetTransform() # get the transform of link 3
    	T3 = robot.GetLinks()[3].GetTransform() # get the transform of link 4
    	T4 = robot.GetLinks()[4].GetTransform() # get the transform of link 5
    	T5 = robot.GetLinks()[5].GetTransform() # get the transform of link 6
    	T6 = robot.GetLinks()[6].GetTransform() # get the transform of link 6
    	handles=[]
    	handles.append(misc.DrawAxes(env,T0,0.3,3))
    	handles.append(misc.DrawAxes(env,T1,0.3,3))
    	handles.append(misc.DrawAxes(env,T2,0.3,3))
    	handles.append(misc.DrawAxes(env,T3,0.3,3))
	handles.append(misc.DrawAxes(env,T4,0.3,3))
    	handles.append(misc.DrawAxes(env,T5,0.3,3))
    	handles.append(misc.DrawAxes(env,T6,0.3,3))
	time.sleep(1)
	
	alpha = [float(alpha[0]*pi/180), float(alpha[1]*pi/180), float(alpha[2]*pi/180), float(alpha[3]*pi/180), float(alpha[4]*pi/180), float(alpha[5]*pi/180)]
	theta=[0,0,0,0,0,0]
	for angle1 in range (min_angles[0], max_angles[0],delta):
		for angle2 in range (min_angles[1], max_angles[1],delta):
			for angle3 in range (min_angles[2], max_angles[2],delta):
				for angle4 in range (min_angles[3], max_angles[3],delta):
					for angle5 in range (min_angles[4], max_angles[4],delta):
						for angle6 in range (min_angles[5], max_angles[5],delta):
							for angle7 in range(-11,-10):
								theta=[angle1*pi/180,angle2*pi/180,angle3*pi/180,angle4*pi/180,angle5*pi/180,angle6*pi/180]

								robot.SetDOFValues(theta,[0,1,2,3,4,5]) # set joint 0 to value 0.5
								T0 = robot.GetLinks()[0].GetTransform() # get the transform of link 1
								T1 = robot.GetLinks()[1].GetTransform() # get the transform of link 2
								T2 = robot.GetLinks()[2].GetTransform() # get the transform of link 3
								T3 = robot.GetLinks()[3].GetTransform() # get the transform of link 4
								T4 = robot.GetLinks()[4].GetTransform() # get the transform of link 5
								T5 = robot.GetLinks()[5].GetTransform() # get the transform of link 6
								T6 = robot.GetLinks()[6].GetTransform() # get the transform of link 6
								handles=[]
								handles.append(misc.DrawAxes(env,T0,0.3,3))
								handles.append(misc.DrawAxes(env,T1,0.3,3))
								handles.append(misc.DrawAxes(env,T2,0.3,3))
								handles.append(misc.DrawAxes(env,T3,0.3,3))
								handles.append(misc.DrawAxes(env,T4,0.3,3))
								handles.append(misc.DrawAxes(env,T5,0.3,3))
								handles.append(misc.DrawAxes(env,T6,0.3,3))
								time.sleep(0.05) 
								
								file_write.write( "\n\nCurrent configuration is" + str([theta[0]*180/pi, theta[1]*180/pi, theta[2]*180/pi, theta[3]*180/pi, theta[4]*180/pi, theta[5]*180/pi, ]) + "\n")	 
								file_write.write( "FORWARD KINEMATICS\n")
								# (i-1)A(i)=
								#		float(cos(theta)	-float(cos(alpha)float(sin(theta)	float(sin(alpha)float(sin(theta)	afloat(cos(theta)
								#		float(sin(theta)	float(cos(alpha)float(cos(theta)	-float(sin(alpha)float(cos(theta)	afloat(sin(theta)
								#		0		float(sin(alpha)		float(cos(alpha)		d
								#		0		0			0			1

								A_01=	[	[float(cos(theta[0])),	-float(cos(alpha[0]))*float(sin(theta[0])),	float(sin(alpha[0]))*float(sin(theta[0])),	float(a[0])*float(cos(theta[0]))],
										[float(sin(theta[0])),	float(cos(alpha[0]))*float(cos(theta[0])),	-float(sin(alpha[0]))*float(cos(theta[0])),	float(a[0])*float(sin(theta[0]))],
										[0,		float(sin(alpha[0])),			float(cos(alpha[0])),			float(d[0])],
										[0,		0,				0,				1]		]

								A_12=	[	[float(cos(theta[1])),	-float(cos(alpha[1]))*float(sin(theta[1])),	float(sin(alpha[1]))*float(sin(theta[1])),	float(a[1])*float(cos(theta[1]))],
										[float(sin(theta[1])),	float(cos(alpha[1]))*float(cos(theta[1])),	-float(sin(alpha[1]))*float(cos(theta[1])),	float(a[1])*float(sin(theta[1]))],
										[0,		float(sin(alpha[1])),			float(cos(alpha[1])),			float(d[1])],
										[0,		0,				0,				1]		]

								A_23=	[	[float(cos(theta[2])),	-float(cos(alpha[2]))*float(sin(theta[2])),	float(sin(alpha[2]))*float(sin(theta[2])),	float(a[2])*float(cos(theta[2]))],
										[float(sin(theta[2])),	float(cos(alpha[2]))*float(cos(theta[2])),	-float(sin(alpha[2]))*float(cos(theta[2])),	float(a[2])*float(sin(theta[2]))],
										[0,		float(sin(alpha[2])),			float(cos(alpha[2])),			float(d[2])],
										[0,		0,				0,				1]		]

								A_34=	[	[float(cos(theta[3])),	-float(cos(alpha[3]))*float(sin(theta[3])),	float(sin(alpha[3]))*float(sin(theta[3])),	float(a[3])*float(cos(theta[3]))],
										[float(sin(theta[3])),	float(cos(alpha[3]))*float(cos(theta[3])),	-float(sin(alpha[3]))*float(cos(theta[3])),	float(a[3])*float(sin(theta[3]))],
										[0,		float(sin(alpha[3])),			float(cos(alpha[3])),			float(d[3])],
										[0,		0,				0,				1]		]

								A_45=	[	[float(cos(theta[4])),	-float(cos(alpha[4]))*float(sin(theta[4])),	float(sin(alpha[4]))*float(sin(theta[4])),	float(a[4])*float(cos(theta[4]))],
										[float(sin(theta[4])),	float(cos(alpha[4]))*float(cos(theta[4])),	-float(sin(alpha[4]))*float(cos(theta[4])),	float(a[4])*float(sin(theta[4]))],
										[0,		float(sin(alpha[4])),			float(cos(alpha[4])),			float(d[4])],
										[0,		0,				0,				1]		]

								A_56=	[	[float(cos(theta[5])),	-float(cos(alpha[5]))*float(sin(theta[5])),	float(sin(alpha[5]))*float(sin(theta[5])),	float(a[5])*float(cos(theta[5]))],
										[float(sin(theta[5])),	float(cos(alpha[5]))*float(cos(theta[5])),	-float(sin(alpha[5]))*float(cos(theta[5])),	float(a[5])*float(sin(theta[5]))],
										[0,		float(sin(alpha[5])),			float(cos(alpha[5])),			float(d[5])],
										[0,		0,				0,				1]		]


								A_02=matrixmult (A_01,A_12)
								A_24=matrixmult (A_23,A_34)
								A_46=matrixmult (A_45,A_56)
								A_04=matrixmult (A_02,A_24)
								A_06=matrixmult (A_04,A_46)

								A_03=matrixmult (A_02,A_23)
								A_36=matrixmult (A_34, A_46)
								A_06=matrixmult (A_03,A_36)

								A_26=matrixmult (A_24, A_46)
								A_16=matrixmult (A_12, A_26)
								
								PositionMatrix= A_06
								file_write.write( "Position Matrix is \n")
								file_write.write(str(A_06[0]))
								file_write.write("\n")
								file_write.write(str(A_06[1]))
								file_write.write("\n")
								file_write.write(str(A_06[2]))
								file_write.write("\n")
								file_write.write(str(A_06[3]))
								file_write.write("\n")
								file_write.write("\n")
								
								A_01=	[	[float(cos(theta[0])),	-float(cos(alpha[0]))*float(sin(theta[0])),	float(sin(alpha[0]))*float(sin(theta[0])),	float(a[0])*float(cos(theta[0]))],
										[float(sin(theta[0])),	float(cos(alpha[0]))*float(cos(theta[0])),	-float(sin(alpha[0]))*float(cos(theta[0])),	float(a[0])*float(sin(theta[0]))],
										[0,		float(sin(alpha[0])),			float(cos(alpha[0])),			float(d[0])],
										[0,		0,				0,				1]		]

								A_12=	[	[float(cos(theta[1])),	-float(cos(alpha[1]))*float(sin(theta[1])),	float(sin(alpha[1]))*float(sin(theta[1])),	float(a[1])*float(cos(theta[1]))],
										[float(sin(theta[1])),	float(cos(alpha[1]))*float(cos(theta[1])),	-float(sin(alpha[1]))*float(cos(theta[1])),	float(a[1])*float(sin(theta[1]))],
										[0,		float(sin(alpha[1])),			float(cos(alpha[1])),			float(d[1])],
										[0,		0,				0,				1]		]

								A_23=	[	[float(cos(theta[2])),	-float(cos(alpha[2]))*float(sin(theta[2])),	float(sin(alpha[2]))*float(sin(theta[2])),	float(a[2])*float(cos(theta[2]))],
										[float(sin(theta[2])),	float(cos(alpha[2]))*float(cos(theta[2])),	-float(sin(alpha[2]))*float(cos(theta[2])),	float(a[2])*float(sin(theta[2]))],
										[0,		float(sin(alpha[2])),			float(cos(alpha[2])),			float(d[2])],
										[0,		0,				0,				1]		]

								A_34=	[	[float(cos(theta[3])),	-float(cos(alpha[3]))*float(sin(theta[3])),	float(sin(alpha[3]))*float(sin(theta[3])),	float(a[3])*float(cos(theta[3]))],
										[float(sin(theta[3])),	float(cos(alpha[3]))*float(cos(theta[3])),	-float(sin(alpha[3]))*float(cos(theta[3])),	float(a[3])*float(sin(theta[3]))],
										[0,		float(sin(alpha[3])),			float(cos(alpha[3])),			float(d[3])],
										[0,		0,				0,				1]		]

								A_45=	[	[float(cos(theta[4])),	-float(cos(alpha[4]))*float(sin(theta[4])),	float(sin(alpha[4]))*float(sin(theta[4])),	float(a[4])*float(cos(theta[4]))],
										[float(sin(theta[4])),	float(cos(alpha[4]))*float(cos(theta[4])),	-float(sin(alpha[4]))*float(cos(theta[4])),	float(a[4])*float(sin(theta[4]))],
										[0,		float(sin(alpha[4])),			float(cos(alpha[4])),			float(d[4])],
										[0,		0,				0,				1]		]

								A_56=	[	[float(cos(theta[5])),	-float(cos(alpha[5]))*float(sin(theta[5])),	float(sin(alpha[5]))*float(sin(theta[5])),	float(a[5])*float(cos(theta[5]))],
										[float(sin(theta[5])),	float(cos(alpha[5]))*float(cos(theta[5])),	-float(sin(alpha[5]))*float(cos(theta[5])),	float(a[5])*float(sin(theta[5]))],
										[0,		float(sin(alpha[5])),			float(cos(alpha[5])),			float(d[5])],
										[0,		0,				0,				1]		]

								A_02=matrixmult (A_01,A_12)
								A_24=matrixmult (A_23,A_34)
								A_46=matrixmult (A_45,A_56)
								A_04=matrixmult (A_02,A_24)
								A_06=matrixmult (A_04,A_46)

								A_03=matrixmult (A_02,A_23)
								A_36=matrixmult (A_34, A_46)
								A_06=matrixmult (A_03,A_36)

								n=[A_06[0][0], A_06[1][0], A_06[2][0] ]
								s=[A_06[0][1], A_06[1][1], A_06[2][1] ]
								approach=[A_06[0][2], A_06[1][2], A_06[2][2] ]

								z4= [A_04[0][2], A_04[1][2], A_04[2][2] ]

								##########	ARM Calculation		###########
								arm_calc = (-d[3]*sin(theta[1]+theta[2]) -a[2]*cos(theta[1]+theta[2]) -a[1]*cos(theta[1]) )
								if arm_calc>=0:
									ARM= 1
								else:
									ARM=-1
								##########	Elbow Calculation		###########
								elbow_calc= d[3]*cos(theta[2]) - a[2]*sin(theta[2])
								if elbow_calc>=0:
									ELBOW=ARM
								else:
									ELBOW=-ARM
								##########	Wrist Calculation		###########
								wrist_calc_s= s[0]*z4[0] + s[1]*z4[1] +s[2]*z4[2]
								wrist_calc_n= n[0]*z4[0] + n[1]*z4[1] +n[2]*z4[2]

								if wrist_calc_s>0:
									WRIST=1
								if wrist_calc_s<0:
									WRIST=-1
								if wrist_calc_s==0:
									if wrist_calc_n >= 0:
										WRIST=1
									else:
										WRIST=-1

								####################################
								file_write.write("\nParameters are\n")
								file_write.write(str(ARM))
								file_write.write("\n") 
								file_write.write(str(ELBOW))
								file_write.write("\n") 
								file_write.write(str(WRIST))
								file_write.write("\n") 
								####################################
								
								# Inverse Kinematics Using HTMs
								file_write.write("SYMBOLIC RESULTS USING HTMs \n")

								#print "theta[0] = 2*atan2(-px +/- sqrt(px*px+py*py-d2*d2),  d2+py)"

								#print "let us define g214 = cos(theta[0])*px + sin(theta[0])*py"
								#print "let us define g224 = -pz"
								#print "let us define d = g214*g214 + g224*g224 -d4*d4 -a3*a3 -a2*a2"
								#print "let us define e = sqrt(4*a2*a2*a3*a3 + 4*a2*a2*d4*d4)"

								#print "theta[2] = 2*atan2(2*a2*d4 +/- sqrt( e*e -d*d), d+2*a2*3)"

								#print "let us define f = g214-a2*cos(theta[2])"
								#print "let us define h = d4*d4 + a3*a3"

								#print "theta[1] =2*atan2((d4*cos(theta[2] - a3*sin(theta[2])) +/- sqrt (d4^2 +a3^2 -f^2) , f+ d4*sin(theta[2]) + a3*cos(theta[2])  )"


								# The calculations
								calculated_thetaA=[-10000,-10000,-10000,-10000,-10000,-10000]
								calculated_thetaB=[-10000,-10000,-10000,-10000,-10000,-10000]

 								#print "editing to make d6 = 0"
								px=A_04[0][3]
								py=A_04[1][3]
								pz=A_04[2][3]

								ax=A_06[0][2]
								ay=A_06[1][2]
								az=A_06[2][2]

								nx=A_06[0][0]
								ny=A_06[1][0]
								nz=A_06[2][0]

								sx=A_06[0][1]
								sy=A_06[1][1]
								sz=A_06[2][1]

								file_write.write("\nINVERSE KINEMATICS \n")
								file_write.write("Using the half angle method \n")

								calculated_theta_halfA=[0,0,0,0,0,0]
								calculated_theta_halfB=[0,0,0,0,0,0]
								
								# theta 1 calculation  -> theta[0]
								calculated_theta_halfA[0] = 180/pi*2*atan2(-px + sqrt(px*px+py*py-d[1]*d[1]),d[1]+py)
								calculated_theta_halfB[0] = 180/pi*2*atan2(-px - sqrt(px*px+py*py-d[1]*d[1]),d[1]+py)

								# theta 3 calculation  -> theta[2]
								g214 = cos(theta[0])*px + sin(theta[0])*py
								g224 = -pz
								d_const = g214*g214 + g224*g224 -d[3]*d[3] -a[2]*a[2] -a[1]*a[1]
								e_const_square= 4*a[1]*a[1]*a[2]*a[2] + 4*a[1]*a[1]*d[3]*d[3]


								calculated_theta_halfA[2] = 180/pi*2*atan2(2*a[1]*d[3] + sqrt(e_const_square -d_const*d_const), d_const+2*a[1]*a[2])
								calculated_theta_halfB[2] = 180/pi*2*atan2(2*a[1]*d[3] - sqrt(e_const_square -d_const*d_const), d_const+2*a[1]*a[2])


								# theta 2 calculation  -> theta[1]
								f=g214-a[1]*cos(theta[2])

								pxx=-(d[3]*cos(theta[2]) -a[2]*sin(theta[2]) )
								pyy=d[3]*sin(theta[2]) +a[2]*cos(theta[2]) + a[1]
								rad=g214
								calculated_theta_halfA[1]=180/pi*2*(atan2( -pxx + sqrt(pxx*pxx+pyy*pyy-rad*rad) , rad + pyy))
								calculated_theta_halfB[1]=180/pi*2*(atan2( -pxx - sqrt(pxx*pxx+pyy*pyy-rad*rad) , rad + pyy))

								# theta 4 calculation  -> theta[3]
								calculated_theta_halfA[3]= 180/pi*atan2( -sin(theta[0])*ax + cos(theta[0])*ay , cos(theta[1]+theta[2])*(cos(theta[0])*ax +sin(theta[0])*ay) - az*(sin(theta[1]+theta[2])) )
								calculated_theta_halfB[3]= 180 + 180/pi*atan2( -sin(theta[0])*ax + cos(theta[0])*ay , cos(theta[1]+theta[2])*(cos(theta[0])*ax +sin(theta[0])*ay) - az*(sin(theta[1]+theta[2])) )

								# theta 5 calculation  -> theta[4]
								calculated_theta_halfA[4]=180/pi*atan2(  cos(theta[3])*(cos(theta[1]+theta[2])*(cos(theta[0])*ax+sin(theta[0])*ay)-sin(theta[1]+theta[2])*az)+sin(theta[3])*(-sin(theta[0])*ax +cos(theta[0])*ay)    ,  sin(theta[1]+theta[2])*(cos(theta[0])*ax + sin(theta[0])*ay) +az*cos(theta[1]+theta[2])  )

								# theta 6 calculation  -> theta[5]
								calculated_theta_halfA[5]=180/pi*atan2(-sin(theta[3])*(cos(theta[1]+theta[2])*(cos(theta[0])*nx+sin(theta[0])*ny)  -sin(theta[1]+theta[2])*nz) + cos(theta[3])*(-sin(theta[0])*nx + cos(theta[0])*ny) , -sin(theta[3])*(cos(theta[1]+theta[2])*(cos(theta[0])*sx+sin(theta[0])*sy)  -sin(theta[1]+theta[2])*sz) + cos(theta[3])*(-sin(theta[0])*sx + cos(theta[0])*sy) )

								# Final Solutions
								file_write.write("INVERSE KINEMATIC SOLUTIONS using half angle method \n")
								file_write.write(str(calculated_theta_halfA[0]) + "     OR     " + str(calculated_theta_halfB[0]) )
								file_write.write("\n")
								file_write.write(str(calculated_theta_halfA[1]) + "     OR     " +  str(calculated_theta_halfB[1]) )
								file_write.write("\n")
								file_write.write(str(calculated_theta_halfA[2]) + "     OR     " + str(calculated_theta_halfB[2]) )
								file_write.write("\n")
								file_write.write(str(calculated_theta_halfA[3]) + "     OR     " + str(calculated_theta_halfB[3]) )
								file_write.write("\n")
								file_write.write(str(calculated_theta_halfA[4]) + "     OR     " + str(calculated_theta_halfB[4]) )
								file_write.write("\n")
								file_write.write(str(calculated_theta_halfA[5]) + "     OR     " + str(calculated_theta_halfB[5]) )
								file_write.write("\n")

								file_write.write("INVERSE KINEMATIC SOLUTIONS using circle method \n")
								# theta 1 calculation  -> theta[0]
								calculated_thetaA[0] = 180/pi*(atan2(py,px) - atan2(d[1],+sqrt(px*px+py*py-d[1]*d[1])))
								calculated_thetaB[0] = 180/pi*(atan2(py,px) - atan2(d[1],-sqrt(px*px+py*py-d[1]*d[1])))

								# theta 3 calculation  -> theta[2]
								g214 = cos(theta[0])*px + sin(theta[0])*py
								g224 = -pz
								d_const = g214*g214 + g224*g224 -d[3]*d[3] -a[2]*a[2] -a[1]*a[1]
								e_const_square= 4*a[1]*a[1]*a[2]*a[2] + 4*a[1]*a[1]*d[3]*d[3]

								calculated_thetaA[2] = 180/pi*(atan2(d_const, sqrt(e_const_square-d_const*d_const)) - atan2(a[2],d[3]))
								calculated_thetaB[2] = 180/pi*(atan2(d_const, -sqrt(e_const_square-d_const*d_const)) - atan2(a[2],d[3]))

								# theta 2 calculation  -> theta[1]
								f=g214-a[1]*cos(theta[2])

								pxx=-(d[3]*cos(theta[2]) -a[2]*sin(theta[2]) )
								pyy=d[3]*sin(theta[2]) +a[2]*cos(theta[2]) + a[1]
								rad=g214
								calculated_thetaA[1]=180/pi*(atan2(pyy,pxx) -atan2(rad,sqrt((pxx*pxx + pyy*pyy)-rad*rad)) )
								calculated_thetaB[1]=180/pi*(atan2(pyy,pxx) -atan2(rad,-sqrt((pxx*pxx + pyy*pyy)-rad*rad)) )

								# theta 4 calculation  -> theta[3]
								calculated_thetaA[3]= 180/pi*atan2( -sin(theta[0])*ax + cos(theta[0])*ay , cos(theta[1]+theta[2])*(cos(theta[0])*ax +sin(theta[0])*ay) - az*(sin(theta[1]+theta[2])) )
								calculated_thetaB[3]= 180 + 180/pi*atan2( -sin(theta[0])*ax + cos(theta[0])*ay , cos(theta[1]+theta[2])*(cos(theta[0])*ax +sin(theta[0])*ay) - az*(sin(theta[1]+theta[2])) )

								# theta 5 calculation  -> theta[4]
								calculated_thetaA[4]=180/pi*atan2(  cos(theta[3])*(cos(theta[1]+theta[2])*(cos(theta[0])*ax+sin(theta[0])*ay)-sin(theta[1]+theta[2])*az)+sin(theta[3])*(-sin(theta[0])*ax +cos(theta[0])*ay)    ,  sin(theta[1]+theta[2])*(cos(theta[0])*ax + sin(theta[0])*ay) +az*cos(theta[1]+theta[2])  )

								# theta 6 calculation  -> theta[5]
								calculated_thetaA[5]=180/pi*atan2(-sin(theta[3])*(cos(theta[1]+theta[2])*(cos(theta[0])*nx+sin(theta[0])*ny)  -sin(theta[1]+theta[2])*nz) + cos(theta[3])*(-sin(theta[0])*nx + cos(theta[0])*ny) , -sin(theta[3])*(cos(theta[1]+theta[2])*(cos(theta[0])*sx+sin(theta[0])*sy)  -sin(theta[1]+theta[2])*sz) + cos(theta[3])*(-sin(theta[0])*sx + cos(theta[0])*sy) )

								# Final Solutions
								file_write.write(str(calculated_thetaA[0]) + "     OR     " + str(calculated_thetaB[0]) )
								file_write.write("\n")
								file_write.write(str(calculated_thetaA[1]) + "     OR     " + str(calculated_thetaB[1]) )
								file_write.write("\n")
								file_write.write(str(calculated_thetaA[2]) + "     OR     " + str(calculated_thetaB[2]) )
								file_write.write("\n")
								file_write.write(str(calculated_thetaA[3]) + "     OR     " + str(calculated_thetaB[3]) )
								file_write.write("\n")
								file_write.write(str(calculated_thetaA[4]) + "     OR     " + str(calculated_thetaB[4]) )
								file_write.write("\n")
								file_write.write(str(calculated_thetaA[5]) + "     OR     " + str(calculated_thetaB[5]) )
								file_write.write("\n")
								
								if ARM==1:
									file_write.write("actual is " + str(calculated_thetaB[0])+ "  =  " + str(calculated_theta_halfB[0])+ "  \n")
									file_write.write("actual is " + str(calculated_thetaB[1])+ "  =  " + str(calculated_theta_halfB[1])+ "  \n")
								else:
									file_write.write("actual is " + str(calculated_thetaA[0])+ "  =  " + str(calculated_theta_halfA[0])+ "  \n")
									file_write.write("actual is " + str(calculated_thetaA[1])+ "  =  " + str(calculated_theta_halfA[1])+ "  \n")
								if ARM*ELBOW==1:
									file_write.write("actual is " + str(calculated_thetaA[2])+ "  =  " + str(calculated_theta_halfB[2])+ "  \n")
								else:
									file_write.write("actual is " + str(calculated_thetaB[2])+ "  =  " + str(calculated_theta_halfA[2])+ "  \n")
								file_write.write("actual is " + str(calculated_thetaA[3])+ "  =  " + str(calculated_theta_halfA[3])+ "  \n")
								file_write.write("actual is " + str(calculated_thetaA[4])+ "  =  " + str(calculated_theta_halfA[4])+ "  \n")
								file_write.write("actual is " + str(calculated_thetaA[5])+ "  =  " + str(calculated_theta_halfA[5])+ "  \n")

								################### GEOMETRIC APPROACH  #########
								file_write.write("\n GEOMETRIC APPROACH \n")
								geometry_calculated_theta = [0,0,0,0,0,0]

								p=[A_04[0][3], A_04[1][3], A_04[2][3]]

								px=p[0]
								py=p[1]
								pz=p[2]

								temp=sqrt(px*px+py*py-d[1]*d[1])
								geometry_calculated_theta[0] = 180/pi*atan2( (-ARM*py*temp-px*d[1])/(px*px+py*py)  ,  (-ARM*px*temp+py*d[1])/(px*px+py*py)     )

								#print geometry_calculated_theta
								# theta2 -> theta[1]
								R=sqrt(px*px+py*py+pz*pz-d[1]*d[1])
								r=sqrt(px*px+py*py-d[1]*d[1])
								sin_alpha=-pz/R
								cos_alpha=-ARM*r/R

								cos_beta=(a[1]*a[1]+R*R-d[3]*d[3]-a[2]*a[2])/(2*a[1]*R)
								sin_beta=sqrt(1-cos_beta*cos_beta)

								sin_theta_2=sin_alpha*cos_beta+(ARM*ELBOW)*cos_alpha*sin_beta
								cos_theta_2=cos_alpha*cos_beta-(ARM*ELBOW)*sin_alpha*sin_beta

								geometry_calculated_theta[1]=180/pi*atan2(sin_theta_2, cos_theta_2)

								# theta3 -> theta[2]
								R=sqrt(px*px+py*py+pz*pz-d[1]*d[1])
								cos_phi = (a[1]*a[1] + d[3]*d[3] + a[2]*a[2] - R*R)/( 2*a[1]*sqrt(d[3]*d[3] + a[2]*a[2]) )
								sin_phi = ARM*ELBOW*sqrt(1-cos_phi*cos_phi)

								sin_beta= d[3] / (sqrt(d[3]*d[3] + a[2]*a[2]) )
								cos_beta= fabs(a[2]) / (sqrt(d[3]*d[3] + a[2]*a[2]) )

								sin_theta_3 = sin_phi*cos_beta - cos_phi*sin_beta
								cos_theta_3 = cos_phi*cos_beta + sin_phi*sin_beta

								geometry_calculated_theta[2] = 180/pi*atan2(sin_theta_3, cos_theta_3)## theta 4, theta 5 and theta 6

								z3= [A_03[0][2], A_03[1][2], A_03[2][2] ]
								cross=[z3[1]*approach[2] - approach[1]*z3[2]  , approach[0]*z3[2] - z3[0]*approach[2]  , z3[0]*approach[1] - z3[1]*approach[0] ]
								cross_mod =  sqrt(cross[0]*cross[0] +cross[1]*cross[1] +cross[2]*cross[2])

								if cross_mod==0:
									sigma = 0
								else:
									cross= [cross[0]/cross_mod, cross[1]/cross_mod, cross[2]/cross_mod]
									s_prod = [s[0]*cross[0] +s[1]*cross[1] +s[2]*cross[2]]
									if (s_prod == 0):
										sigma=[n[0]*cross[0] +n[1]*cross[1] +n[2]*cross[2]]
									else:
										sigma = s_prod

								if sigma>=0:
									M= WRIST
								else:
									M= -WRIST
								temp1= (M* (cos(pi/180*geometry_calculated_theta[0])* approach[1] - sin(pi/180*geometry_calculated_theta[0])* approach[0] ))
								geometry_calculated_theta[3] = 180/pi*atan2( temp1 , M*((cos(pi/180*geometry_calculated_theta[0])*cos(pi/180*geometry_calculated_theta[1]+pi/180*geometry_calculated_theta[2])*approach[0]) + (sin(pi/180*geometry_calculated_theta[0])*cos(pi/180*geometry_calculated_theta[1]+pi/180*geometry_calculated_theta[2])*approach[1]) - (sin(pi/180*geometry_calculated_theta[1]+pi/180*geometry_calculated_theta[2])*approach[2])) )

								C1=cos(pi/180*geometry_calculated_theta[0])
								S1=sin(pi/180*geometry_calculated_theta[0])
								C23=cos(pi/180*geometry_calculated_theta[1] + pi/180*geometry_calculated_theta[2])
								S23=sin(pi/180*geometry_calculated_theta[1] + pi/180*geometry_calculated_theta[2])
								C4=cos(pi/180*geometry_calculated_theta[3])
								S4=sin(pi/180*geometry_calculated_theta[3])

								geometry_calculated_theta[4] = 180/pi*atan2 ( (C1*C23*C4 - S1*S4)*approach[0] + (S1*C23*C4 + C1*S4)*approach[1] - C4*S23*approach[2] , C1*S23*approach[0] + S1*S23*approach[1] + C23*approach[2] ) 
								geometry_calculated_theta[5] = 180/pi*atan2 ((-S1*C4 - C1*C23*S4)*n[0] + (C1*C4 - S1*C23*S4)*n[1] + (S4*S23)*n[2] , (-S1*C4 - C1*C23*S4)*s[0] + (C1*C4 - S1*C23*S4)*s[1] + (S4*S23)*s[2] )
								if FLIP==1:
									geometry_calculated_theta[3] = geometry_calculated_theta[3]+pi
									geometry_calculated_theta[4] = - geometry_calculated_theta[4] 
									geometry_calculated_theta[5] = geometry_calculated_theta[5]+pi
								file_write.write("geormetry calculated theta is\n")
								file_write.write(str(geometry_calculated_theta))
								file_write.write("\nARM = "+str(ARM)+"\nELBOW = "+str(ELBOW)+"\nWRIST = "+str(WRIST)+"\n\n")
	
finally:
	file_write.close()
	env.Destroy()

