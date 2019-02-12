#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import numpy as np
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt

### HELP FUNCTIONS
### creating rotation matrices
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
    TM = Matrix([[           cos(q),           -sin(q),           0,            a], 
                 [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d], 
                 [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d], 
                 [                0,                 0,           0,            1]])
    return TM
    
### plotting function
### plots the received ee position, the fk calculated ee position
### and the error between both of them
def plot(received_pos, fk_ee, ee_e):

    handles = ['fk_ee','fk_rec', 'ee_e']
   
    fig = plt.figure(figsize=(12,9))
    ax = fig.add_subplot(111,projection='3d')
    for i in range(1,len(received_pos),1):    
        ax.scatter(fk_ee[i][0], fk_ee[i][1], fk_ee[i][2], linestyle = '-', c = 'r', marker = 'x')
        ax.scatter(received_pos[i][1], received_pos[i][2], received_pos[i][2], linestyle = '-', c = 'g', marker = 'o' )
        ax.scatter(ee_e[i][0], ee_e[i][1], ee_e[i][2], c = 'b',linestyle = '-', marker = '^')
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.legend(handles)
    ax.set_title('Deviation of EE Position') 
    plt.show()

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
T0_3 = T0_1 * T1_2 * T2_3
T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G   

#Create Correction between DH Table and URDF File
R_Z = create_R_Z(pi)
R_Y = create_R_Y(-pi/2)
R_corr = R_Z * R_Y    
    
# Total Transform between BaseLink and gripper
T0_G = T0_G * R_corr

   
def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print ("No valid poses received")
        return -1
    else:

        # Initialize service response
        joint_trajectory_list = []

        # set to True if you want to plot ee trajectory, fk trajectory and the error
	plotting = False
	
	if plotting == True:	
	    received_ee_pos = np.array([0,0,0])
	    fk_ee = np.array([0,0,0])
	    ee_e = np.array([0,0,0])         
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

        # Populate response for the IK request
        # In the next line replace theta1,theta2...,theta6 by your joint angle variables
        joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
        joint_trajectory_list.append(joint_trajectory_point)
    
        #save data for the plotting
	    if plotting == True:	    
            received_ee_pos = np.vstack([received_ee_pos,[px, py, pz]])
            fk_ee_t = T0_G.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
            fk_ee = np.vstack([fk_ee,[fk_ee_t[0, 3],fk_ee_t[1, 3],fk_ee_t[2, 3]]])  
            ee_e = np.vstack([ee_e,[abs(fk_ee_t[0, 3] - px), abs(fk_ee_t[1, 3] - py), abs(fk_ee_t[2, 3] - pz)]]) 


	rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
	if plotting == True:
	    print 'plotting trajectory'
	    plot(received_ee_pos, fk_ee, ee_e)        
	return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print ("Ready to receive an IK request")
    rospy.spin()

if __name__ == "__main__":
    IK_server()
