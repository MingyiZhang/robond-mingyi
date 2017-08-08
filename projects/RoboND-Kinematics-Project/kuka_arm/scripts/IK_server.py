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
import numpy as np


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
        q1, q2, q3, q4, q5, q6 = symbols('q1:7')

        #########################
        # global constants
        #########################

        # Identity matrix
        I = Matrix([[1, 0, 0],
                    [0, 1, 0],
                    [0, 0, 1]])

        # orientation between frame G and frame gripper_link
        rGg = Matrix([[0, 0, 1],
                      [0, -1, 0],
                      [1, 0, 0]])

        # origin of frame
        # origin = Matrix([[0,0,0,1]]).T

        #########################
        # transform functions
        #########################
        # rotation matrices
        def rot_x(q):
            '''
            rotation around axis X
            '''
            R_x = Matrix([[ 1,              0,        0],
                          [ 0,         cos(q),  -sin(q)],
                          [ 0,         sin(q),  cos(q)]])

            return R_x

        def rot_y(q):
            '''
            rotation around axis Y
            '''
            R_y = Matrix([[ cos(q),        0,  sin(q)],
                          [      0,        1,       0],
                          [-sin(q),        0, cos(q)]])

            return R_y

        def rot_z(q):
            '''
            rotation around axis Z
            '''
            R_z = Matrix([[ cos(q),  -sin(q),       0],
                          [ sin(q),   cos(q),       0],
                          [      0,        0,       1]])

            return R_z

        # translation vectors
        def p_x(d):
            '''
            translation along axis X
            '''
            t = Matrix([[d], [0], [0]])
            return t

        def p_y(d):
            '''
            translation along axis Y
            '''
            t = Matrix([[0], [d], [0]])
            return t

        def p_z(d):
            '''
            translation along axis Z
            '''
            t = Matrix([[0], [0], [d]])
            return t

        # homogeneous transform: put rotation and translation together
        def H(R, p=Matrix([[0], [0], [0]])):
            '''
            R is rotation matrix
            p is translation vector
            '''
            D = R.row_join(p)
            D = D.col_join(Matrix([[0,0,0,1]]))
            return D

        # homogeneous transform between neighbouring frames
        def T(alpha, a, theta, d):
            T = H(rot_x(alpha)) * H(I, p_x(a)) * H(rot_z(theta)) * H(I, p_z(d))
            return T

        # Create Modified DH parameters
        s = {'alpha0': 0,     'a0': 0,      'd1': 0.75,  'theta1': q1,
             'alpha1': -pi/2, 'a1': 0.35,   'd2': 0,     'theta2': q2 - pi/2,
             'alpha2': 0,     'a2': 1.25,   'd3': 0,     'theta3': q3,
             'alpha3': -pi/2, 'a3': -0.054, 'd4': 1.5,   'theta4': q4,
             'alpha4': pi/2,  'a4': 0,      'd5': 0,     'theta5': q5,
             'alpha5': -pi/2, 'a5': 0,      'd6': 0,     'theta6': q6,
             'alpha6': 0,     'a6': 0,      'dG': 0.303, 'thetaG': 0}
        #
        # Homogeneous transform between frames
        T01 = T(s['alpha0'], s['a0'], s['theta1'], s['d1'])
        T12 = T(s['alpha1'], s['a1'], s['theta2'], s['d2'])
        T23 = T(s['alpha2'], s['a2'], s['theta3'], s['d3'])
        T34 = T(s['alpha3'], s['a3'], s['theta4'], s['d4'])
        T45 = T(s['alpha4'], s['a4'], s['theta5'], s['d5'])
        T56 = T(s['alpha5'], s['a5'], s['theta6'], s['d6'])
        T6G = T(s['alpha6'], s['a6'], s['thetaG'], s['dG'])
        TGg = H(rGg)
        #
        # transform from base_point to G
        T0g = T01 * T12 * T23 * T34 * T45 * T56 * T6G * TGg

        #########################
        # Inverse Kinematics functions
        #########################
        def pose_wc(p_g, o_g):
            '''
            get the orientation and position of wrist center (WC) on frame 6
            inputs:
            p_g: numpy array, [x, y, z] position of gripper to origin
            o_g: numpy array, [x, y, z, w] quaternion of gripper to origin
            '''
            # get homogeneous transform of rotation from quaternion og
            T_0g = tf.transformations.quaternion_matrix(o_g)
            # identify pg with the translation part
            T_0g[:3, 3] = p_g

            # np array rGg
            r_Gg = np.array(rGg).astype(np.float32)
            # r06: the 3 rotation from frame 0 to frame 6
            r_06 = T_0g[:3, :3].dot(r_Gg)
            # pwc: position of WC
            x = np.array([[1.], [0.], [0.]])
            p_wc = np.array(p_g).reshape(3,1) - s['dG'] * T_0g[:3,:3].dot(x)
            # return T_0g, r_06, p_wc
            return T_0g, r_06, p_wc

        def P_wc1(p_wc):
            '''
            get position of WC to frame 1
            p_wc: position of WC to frame 0
            '''
            _, T_01 = Q_1(p_wc)
            # get inverse matrix of T_01
            T_01_inv = np.linalg.inv(T_01)
            # 4-vector of p_wc
            p_wc_4 = np.append(p_wc, 1).reshape(4, 1)
            # p_wc1: WC position to frame 1
            p_wc1 = T_01_inv.dot(p_wc_4)
            # p_wc1 = p_wc1[:3]
            return p_wc1

        def Q_1(p_wc):
            '''
            calculate q1
            inputs:
            p_wc: position of WC to frame 0
            '''
            q_1 = atan2(p_wc[1], p_wc[0]).evalf()
            T_01 = np.array(T(s['alpha0'], s['a0'], s['theta1'], s['d1']).evalf(subs={q1: q_1})).astype(np.float32)
            return q_1, T_01

        def Q_2(p_wc1):
            '''
            calculate q2
            inputs:
            p_wc1: WC position to frame 1, numpy array shape=(4, 1),
            '''
	    k34 = sqrt(s['d4'] ** 2 + s['a3'] ** 2)
            k24 = sqrt((p_wc1[0] - s['a1']) ** 2 + p_wc1[2] ** 2)
            cosq23 = (s['a2']**2 + k24**2 - k34**2) / (2 * s['a2'] * k24)
            q23 = atan2(sqrt(1 - cosq23**2), cosq23)

            q21 = atan2(p_wc1[2], p_wc1[0]-s['a1'])

            q_2 = (pi/2 - q23 - q21).evalf()
            T_12 = np.array(T(s['alpha1'], s['a1'], s['theta2'], s['d2']).evalf(subs={q2: q_2})).astype(np.float32)
            return q_2, T_12

        def Q_3(p_wc1):
            '''
            calculate q3
            inputs:
            p_wc1: WC position to frame 1, numpy array shape=(4, 1),
            '''
            q34 = atan2(s['d4'], s['a3'])
            k34 = sqrt(s['d4'] ** 2 + s['a3'] ** 2)
            k24 = sqrt((p_wc1[0] - s['a1']) ** 2 + p_wc1[2] ** 2)

            cosq24 = (s['a2']**2 + k34**2 - k24**2) / (2 * s['a2'] * k34)
            q24 = atan2(sqrt(1 - cosq24**2), cosq24)

            q_3 = (-q24 + q34).evalf()
            T_23 = np.array(T(s['alpha2'], s['a2'], s['theta3'], s['d3']).evalf(subs={q3: q_3})).astype(np.float32)
            return q_3, T_23

        def Q_456(r_03, r_06):
            '''
            calculate q4, q5, q6
            inputs:
            r_03: rotation from frame 3 to frame 0, numpy array, shape=(3,3)
            r_06: rotation from frame 6 to frame 6, numpy array, shape=(3,3)
            '''
            r_03_inv = np.linalg.inv(r_03)
            r_36 = r_03_inv.dot(r_06)

            r21, r22, r23 = r_36[1, :]
            r13, r33 = r_36[[0,2], 2]

            q_4 = atan2(r33, -r13).evalf()
            q_5 = atan2(sqrt(r21**2 + r22**2), r23).evalf()
            q_6 = atan2(-r22, r21).evalf()
            T_34 = np.array(T(s['alpha3'], s['a3'], s['theta4'], s['d4']).evalf(subs={q4: q_4})).astype(np.float32)
            T_45 = np.array(T(s['alpha4'], s['a4'], s['theta5'], s['d5']).evalf(subs={q5: q_5})).astype(np.float32)
            T_56 = np.array(T(s['alpha5'], s['a5'], s['theta6'], s['d6']).evalf(subs={q6: q_6})).astype(np.float32)
            return q_4, q_5, q_6, T_34, T_45, T_56
    	#


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

            p_g = np.array([px, py, pz])
            o_g = np.array([req.poses[x].orientation.x, req.poses[x].orientation.y,
                            req.poses[x].orientation.z, req.poses[x].orientation.w])

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
	        # Calculate joint angles using Geometric IK method
            T_0g, r_06, p_wc = pose_wc(p_g, o_g)
            q_1, T_01 = Q_1(p_wc)
            p_wc1 = P_wc1(p_wc)
            q_2, T_12 = Q_2(p_wc1)
            q_3, T_23 = Q_3(p_wc1)
            T_03 = T_01.dot(T_12).dot(T_23)
            r_03 = T_03[:3, :3]
            q_4, q_5, q_6, T_34, T_45, T_56 = Q_456(r_03, r_06)

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [q_1, q_2, q_3, q_4, q_5, q_6]
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
