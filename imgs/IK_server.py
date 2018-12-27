#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		
        ### Your FK code here
        # Create symbols
	#
	#
	alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
	theta0, theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta0:7')
	a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
	d0, d1, d2, d3, d4, d5, d6 = symbols('d0:7')
	
	s = {alpha0:     0, a0:      0, d0:  0.75,
	     alpha1: -pi/2, a1:   0.35, d1:     0, theta1: theta1 - pi/2,
	     alpha2:     0, a2:   1.25, d2:     0,
	     alpha3: -pi/2, a3: -0.054, d3:   1.5,
	     alpha4:  pi/2, a4:      0, d4:     0,
	     alpha5: -pi/2, a5:      0, d5:     0,
	     alpha6:     0, a6:      0, d6: 0.303, theta6: 0}
	# Create Modified DH parameters
	#
	#            
	# Define Modified DH Transformation matrix
	#
	#
	T0_1 = Matrix([ [           cos(theta0),             -sin(theta0), 	      0, 	      a0],
			[sin(theta0)*cos(alpha0), cos(theta0)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d0],
			[sin(theta0)*sin(alpha0), cos(theta0)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d0],
			[		       0, 		        0, 	      0, 	       1] ])
	T0_1 = T0_1.subs(s)
	
	T1_2 = Matrix([ [            cos(theta1),            -sin(theta1),            0,              a1],
                        [sin(theta1)*cos(alpha1), cos(theta1)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d1],
                        [sin(theta1)*sin(alpha1), cos(theta1)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d1],
                        [                      0,                       0,            0,               1] ])
	T1_2 = T1_2.subs(s)
	
	T2_3 = Matrix([ [            cos(theta2),            -sin(theta2),            0,              a2],
                        [sin(theta2)*cos(alpha2), cos(theta2)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d2],
                        [sin(theta2)*sin(alpha2), cos(theta2)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d2],
                        [                      0,                       0,            0,               1] ])
	T2_3 = T2_3.subs(s)
	
	T3_4 = Matrix([ [            cos(theta3),            -sin(theta3),            0,              a3],
                        [sin(theta3)*cos(alpha3), cos(theta3)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d3],
                        [sin(theta3)*sin(alpha3), cos(theta3)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d3],
                        [                      0,                       0,            0,               1] ])
	T3_4 = T3_4.subs(s)
	
	T4_5 = Matrix([ [            cos(theta4),            -sin(theta4),            0,              a4],
                        [sin(theta4)*cos(alpha4), cos(theta4)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d4],
                        [sin(theta4)*sin(alpha4), cos(theta4)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d4],
                        [                      0,                       0,            0,               1] ])
	T4_5 = T4_5.subs(s)
	
	T5_6 = Matrix([ [            cos(theta5),            -sin(theta5),            0,              a5],
                        [sin(theta5)*cos(alpha5), cos(theta5)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d5],
                        [sin(theta5)*sin(alpha5), cos(theta5)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d5],
                        [                      0,                       0,            0,               1] ])
	T5_6 = T5_6.subs(s)
	
	T6_G = Matrix([ [            cos(theta6),            -sin(theta6),            0,              a6],
                        [sin(theta6)*cos(alpha6), cos(theta6)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d6],
                        [sin(theta6)*sin(alpha6), cos(theta6)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d6],
                        [                      0,                       0,            0,               1] ])
	T6_G = T6_G.subs(s)
	
	# Create individual transformation matrices
	#
	#
	
	# Extract rotation matrices from the transformation matrices
	#
	#
	T0_2 = T0_1 * T1_2
	T0_3 = T0_2 * T2_3
	T0_4 = T0_3 * T3_4
	T0_5 = T0_4 * T4_5
	T0_6 = T0_5 * T5_6
	T0_G = T0_6 * T6_G

	R_z = Matrix([  [ cos(pi),-sin(pi), 0, 0],
			[ sin(pi), cos(pi), 0, 0],
			[	0,	 0, 1, 0],
			[	0,	 0, 0, 1]  ])

	R_y = Matrix([  [ cos(-pi/2), 0, sin(-pi/2), 0],
                        [	   0, 1, 	  0, 0],
                        [-sin(-pi/2), 0, cos(-pi/2), 0],
                        [	   0, 0, 	  0, 1]  ])

	R_corr = R_z * R_y

	T_final = T0_G * R_corr
        ###

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
     
            ### Your IK code here 
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    #
	    #
	    R_x = Matrix([[1, 	      0, 	  0, 0],
			  [0, cos(roll), -sin(roll), 0],
			  [0, sin(roll),  cos(roll), 0],
			  [0, 	      0, 	  0, 1]])

	    R_y = Matrix([[ cos(pitch), 0, sin(pitch), 0],
                          [	     0, 1, 	    0, 0],
                          [-sin(pitch), 0, cos(pitch), 0],
			  [	     0, 0, 	    0, 1]])

	    R_z = Matrix([[cos(yaw), -sin(yaw), 0, 0],
                          [sin(yaw),  cos(yaw), 0, 0],
                          [	  0, 	     0, 1, 0],
			  [	  0, 	     0, 0, 1]])

	    Rrpy = R_z * R_y * R_x * R_corr

	    # Using the math given in the IK lesson, we have:
	    nx = Rrpy[0, 2]
	    ny = Rrpy[1, 2]
	    nz = Rrpy[2, 2]

	    l = 0.303

	    # wx, wy, and wz represent the WC position.
	    wx = px - l * nx
	    wy = py - l * ny
	    wz = pz - l * nz

	    # Calculate joint angles using Geometric IK method
	    #
	    #

	    # theta0 is easily given by the WC position.
	    new_theta0 = atan2(wy, wx)

	    # Find the position of joint 2 given theta0 and the distances defined in the DH parameters.
	    j2_x = 0.35 * cos(new_theta0)
	    j2_y = 0.35 * sin(new_theta0)
	    j2_z = 0.75

	    # Define a, b, and c to apply the Cosine Law.
	    a = 1.25
	    b = 1.501
	    c = sqrt((j2_x - wx)**2 + (j2_y - wy)**2 + (j2_z - wz)**2) # c is the distance from joint 2 to the WC.

	    # These xi angles represent the angles formed by the links; NOT equal to the theta's I'm looking for.
	    xi_1 = acos((a**2 + c**2 - b**2) / (2 * a * c))
	    xi_2 = acos((a**2 + b**2 - c**2) / (2 * a * b))
	    xi_3 = acos((b**2 + c**2 - a**2) / (2 * b * c))

	    # The difference in x and y between the WC and the joint.
	    j_w_x = wx - j2_x
	    j_w_y = wy - j2_y

	    # The distance between the WC and joint 2 measured along the xy plane.
	    j_w_xy = sqrt(j_w_x**2 + j_w_y**2)

	    # My thetas are given by subtracting them to pi/2.
	    new_theta1 = pi/2 - (xi_1 + atan2(wz - j2_z, j_w_xy))
	    new_theta2 = pi/2 - xi_2

	    # Why worry about the orientation of the end-effector when we are too far away from the goal for it to matter anyway?
	    # This way, the calculations are much faster!
	    if abs(x - len(req.poses)) <= 5:
	    	S = T0_3.evalf(subs={theta0: new_theta0, theta1: new_theta1, theta2: new_theta2}).inv("LU") * Rrpy

    	    	new_theta3 = atan2(S[2, 2], -S[0, 2])
    	    	new_theta4 = atan2(sqrt(S[0, 2]**2 + S[2, 2]**2), S[1, 2])
    	    	new_theta5 = atan2(-S[1, 1], S[1, 0])
	    else:
	    	new_theta3 = 0
	    	new_theta4 = 0
	    	new_theta5 = 0
            ###
		
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [new_theta0, new_theta1, new_theta2, new_theta3, new_theta4, new_theta5]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
