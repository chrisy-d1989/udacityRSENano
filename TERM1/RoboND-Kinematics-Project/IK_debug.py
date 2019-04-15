from sympy import *
from time import time
from mpmath import radians
import tf
import numpy as np
import rospy
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}

# Create symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

# Create Modified DH parameters
s = {alpha0:     0, a0:      0, d1:  0.75, q1:      q1,
	alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2,
	alpha2:     0, a2:   1.25, d3:     0, q3:      q3,
	alpha3: -pi/2, a3: -0.054, d4:  1.50, q4:      q4,
	alpha4:  pi/2, a4:      0, d5:     0, q5:      q5,
	alpha5: -pi/2, a5:      0, d6:     0, q6:      q6,
	alpha6:     0, a6:      0, d7: 0.303, q7:       0}

### Helper Functions
def create_R_X(angle):
    '''
    Compute Rotation around fix x axis

    Keyword arguments: 
    angle: rotation angle

    Return values:
    rotation matrix
    '''
    R_X = Matrix([[ 1,          0,           0, 0], 
                  [ 0, cos(angle), -sin(angle), 0], 
                  [ 0, sin(angle),  cos(angle), 0], 
                  [ 0,          0,           0, 1]]) 
    return R_X

def create_R_Y(angle):
    '''
    Compute Rotation around fix y axis

    Keyword arguments: 
    angle: rotation angle

    Return values:
    rotation matrix
    '''
    R_Y = Matrix([[  cos(angle), 0, sin(angle), 0], 
                  [           0, 1,          0, 0], 
                  [ -sin(angle), 0, cos(angle), 0], 
                  [           0, 0,          0, 1]])     

    return R_Y

def create_R_Z(angle):
    '''
    Compute Rotation around fix z axis

    Keyword arguments: 
    angle: rotation angle

    Return values:
    rotation matrix
    '''
    R_Z = Matrix([[ cos(angle), -sin(angle), 0, 0], 
                  [ sin(angle),  cos(angle), 0, 0], 
                  [          0,           0, 1, 0], 
                  [          0,           0, 0, 1]])
    return R_Z

def create_TM(alpha, a, d, q):
    '''
    Compute Transformation matrix

    Keyword arguments: 
    DH modified arguments as input arguments 

    Return values:
    transformation matrix
    '''
    TM = Matrix([[           cos(q),           -sin(q),           0,            a0], 
                 [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d], 
                 [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d], 
                 [                0,                 0,           0,            1]])
    TM = TM.subs(s)
    return TM
### plotting function
def plot(T0_G, px, py, pz, theta1, theta2, theta3, theta4, theta5, theta6):
    handles = ['fk_ee','fk_rec', 'ee_e']

    fk_ee = T0_G.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
    fk_ee = [fk_ee[0, 3],fk_ee[1, 3],fk_ee[2, 3]]
    fk_rec = [px, py, pz]
    ee_x_e = abs(fk_ee[0] - px)
    ee_y_e = abs(fk_ee[1] - py)
    ee_z_e = abs(fk_ee[2] - pz)
    ee_e = [ee_x_e, ee_y_e, ee_z_e]
    ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
    
    fig = plt.figure(figsize=(12,9))
    ax = fig.add_subplot(111,projection='3d')
    ax.scatter(fk_ee[0], fk_ee[1], fk_ee[2], c = 'r', marker = 'x')
    ax.scatter(fk_rec[0], fk_rec[1], fk_rec[2], c = 'g', marker = 'o' )
    ax.scatter(ee_e[0], ee_e[1], ee_e[2	], c = 'b', marker = '^')
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.legend(handles)
    ax.set_title('Deviation of EE Position') 
    
    plt.show()
# Define Modified DH Transformation matrix
# Create individual transformation matrices
T0_1 = create_TM(alpha0, a0, d1, q1).subs(s)
T1_2 = create_TM(alpha1, a1, d2, q2).subs(s)
T2_3 = create_TM(alpha2, a2, d3, q3).subs(s)
T3_4 = create_TM(alpha3, a3, d4, q4).subs(s)
T4_5 = create_TM(alpha4, a4, d5, q5).subs(s)
T5_6 = create_TM(alpha5, a5, d6, q6).subs(s)
T6_G = create_TM(alpha6, a6, d7, q7).subs(s)

# Extract rotation matrices from the transformation matrices
# Transformation from link n to n+1:Tn_(n+1)
#T0_2 = simplify (T0_1 * T1_2)
#T0_3 = simplify (T0_2 * T2_3)
#T0_4 = simplify (T0_3 * T3_4)
#T0_5 = simplify (T0_4 * T4_5)
#T0_6 = simplify (T0_5 * T5_6)
#T0_G = simplify (T0_6 * T6_G)
T0_3 = T0_1 * T1_2 * T2_3
T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G   

#Create Correction between DH Table and URDF File
R_Z = create_R_Z(pi)
R_Y = create_R_Y(-pi/2)
R_corr = R_Z * R_Y    

# Total Transform between BaseLink and gripper
T0_G = simplify(T0_G * R_corr)    

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print ("No valid poses received")
        return -1
    else:     
   
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            # Compensate for rotation discrepancy between DH parameters and Gazebo
            
            # Rotation for calculating Rrpy with roll, pitch, yaw from rviz
            
            R_Zy = create_R_Z(yaw)
            R_Yp = create_R_Y(pitch)
            R_Xr = create_R_X(roll)
            Rrpy = R_Zy * R_Yp * R_Xr * R_corr

            # Calculate wrist positions wcx, wcy, wcz
            n_x = Rrpy[0, 2]
            n_y = Rrpy[1, 2]
            n_z = Rrpy[2, 2]
            
            # Calculate WC(wrist center) position x,y, z with wc = p - (d6 + l) * n
            # writeup file for variable description
            wc_x = px - (s[d6] + s[d7]) * n_x
            wc_y = py - (s[d6] + s[d7]) * n_y
            wc_z = pz - (s[d6] + s[d7]) * n_z
            A = s[d4]
            C = s[a2]
            b_z = wc_z - s[d1]
            b_xy = sqrt(wc_x * wc_x + wc_y * wc_y) - s[a1]
            B = sqrt(b_z * b_z + b_xy * b_xy)
            
            # Calculate angles with cosine law
            # writeup file for variable description
            a = acos((B * B + C * C - A * A) / (2*B*C))
            b = acos((A * A + C * C - B * B) / (2*A*C))
            c = acos((A * A + B * B - C * C) / (2*A*B))
            
	    #sag angle in link4         
            sag = atan2(s[a3], s[d4])
                      
            theta1 = atan2(wc_y, wc_x)
            theta2 = pi/2 - a - atan2(b_z, b_xy) 
            theta3 = pi/2 - b - sag

            # Calculate rotation from link 3 to 6
            R0_3 = T0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            R3_6 = R0_3.T * Rrpy

	    # Calculate joint angles using Geometric IK method
  
            theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
            theta5 = atan2(sqrt(R3_6[0, 2] * R3_6[0, 2] + R3_6[2, 2] * R3_6[2, 2]), R3_6[1, 2]) 
            theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])            

            return (theta1, theta2, theta3, theta4, theta5, theta6, wc_x, wc_y, wc_z, px, py, pz)

def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ## 

 
    theta1, theta2, theta3, theta4, theta5, theta6, wc_x, wc_y, wc_z, px, py ,pz = handle_calculate_IK(req)
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    your_ee = T0_G.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [wc_x, wc_y, wc_z] # <--- Load your calculated WC values in this array
    your_ee = [your_ee[0, 3],your_ee[1, 3],your_ee[2, 3]] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)


    return (px, py, pz, theta1, theta2, theta3, theta4, theta5, theta6)

if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 2

    px, py, pz, theta1, theta2, theta3, theta4, theta5, theta6 = test_code(test_cases[test_case_number])
    plot(T0_G, px, py, pz, theta1, theta2, theta3, theta4, theta5, theta6)
