## Project: Kinematics Pick & Place

[//]: # (Image References)

[image1]: ./doc/fk_1.png
[image2]: ./doc/fk_2.png
[image3]: ./doc/ik.png
[image4]: ./doc/scenery_pap.png
[image5]: ./doc/default_gzclient_camera(1)-2018-12-15T07_19_15.972764.jpg
[image6]: ./doc/trajectory_reaching2.png
[image7]: ./doc/trajectory_dropping.png
##Contents
* Introduction
* Kinematic Analysis
	- Environmental Setup with forward_kinematics demo
	- Derive DH parameters
	- Create transform matrices
	- Inverse Kinematics
* Project Implementation
 	- Tests
 	- Results
 	- Improvements

## Introduction
This project based on [Udacity's Pick and Place Project](https://github.com/udacity/RoboND-Kinematics-Project) uses ROS and its Gazebo and Rviz simulation plattforms to programm a Kuka KR210 6 degree of freedom serial manipulator to pick up a can from a shelf and drop it into a box next to the robot arm. 

The goal is to pick up the can which is randomly placed in the shelf and put it in a box located besides to robot arm. This should be achieved by implementing the forward and inverse kinematics, to decrease the deviation from the planned trajectory to the actual moved trajectory. Figure 1 shows the Kuka arm and the scenery with the box and the shelf.

I followed the [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points to solve this project step by step.

![alt text][image4]
*Figure 1 - Scenery of Pick and Place Project*

## Kinematic Analysis 
###Environmental Setup

The project uses ROS running on Ubuntu 16.04 with the following tools:

* Gazebo, a 3D simulator of ROS
* RViz, a 3d visualizer for sensor data and robot model analyzis
* MoveIt!, a ROS based software framework for motion planning, kinnematics and robot control
* VMWare, as the projects virtual machine

See [Udacity's Pick and Place Project Repo](https://github.com/udacity/RoboND-Kinematics-Project) for specific explanation of setting up the environment and do tests with the demo run of the project.

### Derive DH Parameters

To derive the DH parameters the following 9 steps were performed: 

1  Label joints from 1 to n = 6
2. Label each link from 0 to n = 6
3. Define z-axes as the joint axes (joints 2, 3, and 5 are all parallel while joints 4 and 6 are coincident)
4. Define x-axes as the common normals
5. Define reference frame origins for each joint
6. Define the x-axes as the common normals between z<sub>i-1</sub> and z<sub>i</sub>
7. Define the origin of frame {i} as the intersection of x<sub>i</sub> with z<sub>i</sub>
8. Add a fixed frame rigidly attached to link 6 for the gripper or EE (Note: the EE reference frame O<sub>EE</sub> differs from the link 6 reference frame of O<sub>4</sub>, O<sub>5</sub>, O<sub>6</sub> only by a translation along z<sub>6</sub>)
9. Label all non-zero DH parameters

Figure 2 displays the named joints and joint axis.

![alt text][image2]
*Figure 2 - Description of Joint Names and joint axis*

The following table shows the relative location of joint{i-1} to joint{i} derived from the kr210.urdf.xacro file in the [project repository](https://github.com/udacity/RoboND-Kinematics-Project). 

Joint name | Parent Link | Child Link | x[m] | y[m] | z [m] | roll | pitch | yaw
--- | --- | --- | --- | --- | --- | --- | --- | ---  
joint_1 | base_link | link_1 | 0 | 0 | 0.33 | 0 | 0 | 0 |
joint_2 | link_1| link_2 | 0.35 | 0 | 0.42 | 0 | 0 | 0 |
joint_3 | link_2 | link_3 | 0 | 0 | 1.25 | 0 | 0 | 0 |
joint_4 |  link_3 | link_4 | 0.96 | 0 | -0.054 | 0 | 0 | 0 |
joint_5| link_4 | link_5 | 0.54 | 0 | 0 | 0 | 0 | 0 |
joint_6 | link_5 | link_6 | 0.193 | 0 | 0 | 0 | 0 | 0 |
gripper-joint | link_6 | gripper_link | 0.11 | 0 | 0.33 | 0 | 0 | 0 |

The following DH Table is derived from the table above and figure 2. This table will be used in the project implemention section to calculate the current robot arm pose.

Links | alpha(i-1)[rad] | a(i-1)[m] | d(i-1)[m] | theta(i)[rad]
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.33 + 0.42 = 0.75 | qi
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 0 | 1.25 | q3
3->4 |  -pi/2 | 0 | 0.96 + 0.54 = 1.5| q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.193 + 0.11 = 0.303 | 0

### Create Transformation Matrices

```python
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
        
# Create Transformation
TM = Matrix([[           cos(q),           -sin(q),           0,            a], 
                 [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d], 
                 [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d], 
                 [                0,                 0,           0,            1]])
                 
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
T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G   
```
### Inverse Kinematics
Inverse Kinematics is computing the joint angles when knowing the end effector's position and orientation. Joint 1,2 and 3 control the position of the spherical wrist consisting of joints 4, 5 and 6. The problem is devided into a position and a orientation kinematics problem. We calculate the position of the wrist center and the orientation of ` theta1`  to ` theta3` with the help of the position kinematics with the following equations:

$$ w_x = p_x - (d_6 + l)  n_x $$
$$ w_y = p_y - (d_6 + l)  n_y $$
$$ w_z = p_z - (d_6 + l)  n_z $$

Where p<sub>x</sub>, p<sub>y</sub>, p<sub>z</sub>  are the end-effector positions and w<sub>x</sub>, w<sub>y</sub>, w<sub>z</sub> are the wrist positions, d<sub>6</sub> comes from the dh table and l is the end-effector length. n<sub>x</sub>, n<sub>y</sub> and n<sub>z</sub> is the third column of the homogenous transform.

And figure 4, that shows the visualization of joint 2, 3 and the wrist center to calculate`theta4` to `theta6`.

![alt text][image3]
*Figure 4 - Calucation of Inverse Kinematics*

The inverse orientation kinematics solution gives us `theta4` to `theta6` with the help of 

$$ R^0_3 =  R^0_1 *  R^1_2 *  R^2_3 $$
$$ R^0_3 =  R^0_1 *  R^1_2 *  R^2_3 * R^3_4 *  R^4_5 *  R^5_6 $$
$$ R^3_6 =  (R^0_3)^{-1} *  R^0_6  $$

The exact applied math of the inverse position kinematics and inverse orientation kinematics can be found in the code of `IK_server.py` and `IK_debug.py`.

### Project Implementation

#### Tests

Goal of the project is to place at least 8 blue cans into the box on the left side of the robot arm with the derived kinematics equation from the first section within 10 cycles. Therefore, the equations were implemented in the  `IK_server.py` file and was debugged with the help of `IK_debug.py`. 
Figure 5 shows the end of a pick and place cycle with a successfull finish by placing the blue can in the box.

![alt text][image5]
*Figure 5- Successfull finish of a pick and place cycle*

#### Results
The implemented code in `IK_debug.py` works fine and has a succedding rate of 90 %. While doing 10 cycles it placed 9 times the blue can into the box on the right side of the robot arm. Figure 6 shows the position of the end-effector with the received data(fk_rec), the calculated position (fk_ee) and the deviation between both of them.

![alt text][image6]
*Figure 6 - Received EE Position, FK EE Position and Error between both*

The time that is needed to complete one cycle depends on the location of the can in the shelf. Therefore, on cycle is between ~40 seconds and more than a minute.

#### Improvements

The accuracy of the code, especially for `theta3`  to ` theta6`, can be improved as well as the duration of completing one cycle can be decreased with a much more efficient code. 