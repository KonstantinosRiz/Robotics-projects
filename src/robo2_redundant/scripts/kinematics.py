#!/usr/bin/env python3

"""
Compute state space kinematic matrices for xArm7 robot arm (5 links, 7 joints)
"""

import numpy as np
import math
from sympy import *

class xArm7_kinematics():
    def __init__(self):

        self.l1 = 0.267
        self.l2 = 0.293
        self.l3 = 0.0525
        self.l4 = 0.3512
        self.l5 = 0.1232

        self.theta1 = 0.2225 #(rad) (=12.75deg)
        self.theta2 = 0.6646 #(rad) (=38.08deg)

        pass

    # def compute_jacobian(self, r_joints_array):

    #     Q1, Q2, Q3, Q4, Q5, Q6, Q7 = r_joints_array

    #     var('q1 q2 q3 q4 q5 q6 q7 t1 t2 l1 l2 l3 l4 l5')

    #     tf = np.matrix([[cos(q1) , -sin(q1) , 0 , 0],
    #                     [sin(q1) , cos(q1) , 0 , 0],
    #                     [0 , 0 , 1 , l1],
    #                     [0 , 0 , 0 , 1]])


    #     tf_A12 = np.matrix([[cos(q2) , -sin(q2) , 0 , 0],
    #                         [0 , 0 , 1 , 0],
    #                         [-sin(q2) , -cos(q2) , 0 , 0],
    #                         [0 , 0 , 0 , 1]])

    #     tf_A23 = np.matrix([[cos(q3) , -sin(q3) , 0 , 0],
    #                         [0 , 0 , -1 , -l2],
    #                         [sin(q3) , cos(q3) , 0 , 0],
    #                         [0 , 0 , 0 , 1]])

    #     tf_A34 = np.matrix([[cos(q4) , -sin(q4) , 0 , l3],
    #                         [0 , 0 , -1 , 0],
    #                         [sin(q4) , cos(q4) , 0 , 0],
    #                         [0 , 0 , 0 , 1]])

    #     tf_A45 = np.matrix([[cos(q5) , -sin(q5) , 0 , l4 * sin(t1)],
    #                         [0 , 0 , -1 , - l4 * cos(t1)],
    #                         [sin(q5) , cos(q5) , 0 , 0],
    #                         [0 , 0 , 0 , 1]])

    #     tf_A56 = np.matrix([[cos(q6) , -sin(q6) , 0 , 0],
    #                         [0 , 0 , -1 , 0],
    #                         [sin(q6) , cos(q6) , 0 , 0],
    #                         [0 , 0 , 0 , 1]])

    #     tf_A67 = np.matrix([[cos(q7) , -sin(q7) , 0 , l5 * sin(t2)],\
    #                         [0 , 0 , 1 , l5 * cos(t2)],\
    #                         [-sin(q7) , -cos(q7) , 0 , 0],\
    #                         [0 , 0 , 0 , 1]])

    #     # result = np.dot(tf,tf_A12)
    #     # result2 = np.dot(result,tf_A23)
    #     # result3 = np.dot(result2,tf_A34)
    #     # result4 = np.dot(result3,tf_A45)
    #     # result5 = np.dot(result4,tf_A56)
    #     # result6 = np.dot(result5,tf_A67)

    #     # P = result6[0:3, 3]
    #     # print('Not simplified', P)
    #     # P = simplify(P)
    #     # print('Simplified', result6[0,3])

    #     result = np.dot(tf,tf_A12)
    #     result3 = np.dot(result,tf_A23)
    #     result4 = np.dot(result3,tf_A34)
    #     result5 = np.dot(result4,tf_A45)
    #     result6 = np.dot(result5,tf_A56)
    #     result7 = np.dot(result6,tf_A67)

    #     # P = result7[0:3, 3]

    #     J_11 = diff(result7[0,3], q1)
    #     J_11 = J_11.subs([(q1, Q1), (q2, Q2), (q3, Q3), (q4, Q4), (q5, Q5), (q6, Q6), (q7, Q7), (t1, self.theta1), (t2, self.theta2), (l1, self.l1), (l2, self.l2), (l3, self.l3), (l4, self.l4), (l5, self.l5)])
    #     J_12 = diff(result7[0,3], q2)
    #     J_12 = J_12.subs([(q1, Q1), (q2, Q2), (q3, Q3), (q4, Q4), (q5, Q5), (q6, Q6), (q7, Q7), (t1, self.theta1), (t2, self.theta2), (l1, self.l1), (l2, self.l2), (l3, self.l3), (l4, self.l4), (l5, self.l5)])
    #     J_13 = diff(result7[0, 3], q3)
    #     J_13 = J_13.subs([(q1, Q1), (q2, Q2), (q3, Q3), (q4, Q4), (q5, Q5), (q6, Q6), (q7, Q7), (t1, self.theta1), (t2, self.theta2), (l1, self.l1), (l2, self.l2), (l3, self.l3), (l4, self.l4), (l5, self.l5)])
    #     J_14 = diff(result7[0, 3], q4)
    #     J_14 = J_14.subs([(q1, Q1), (q2, Q2), (q3, Q3), (q4, Q4), (q5, Q5), (q6, Q6), (q7, Q7), (t1, self.theta1), (t2, self.theta2), (l1, self.l1), (l2, self.l2), (l3, self.l3), (l4, self.l4), (l5, self.l5)])
    #     J_15 = diff(result7[0, 3], q5)
    #     J_15 = J_15.subs([(q1, Q1), (q2, Q2), (q3, Q3), (q4, Q4), (q5, Q5), (q6, Q6), (q7, Q7), (t1, self.theta1), (t2, self.theta2), (l1, self.l1), (l2, self.l2), (l3, self.l3), (l4, self.l4), (l5, self.l5)])
    #     J_16 = diff(result7[0, 3], q6)
    #     J_16 = J_16.subs([(q1, Q1), (q2, Q2), (q3, Q3), (q4, Q4), (q5, Q5), (q6, Q6), (q7, Q7), (t1, self.theta1), (t2, self.theta2), (l1, self.l1), (l2, self.l2), (l3, self.l3), (l4, self.l4), (l5, self.l5)])
    #     J_17 = diff(result7[0, 3], q7)
    #     J_17 = J_17.subs([(q1, Q1), (q2, Q2), (q3, Q3), (q4, Q4), (q5, Q5), (q6, Q6), (q7, Q7), (t1, self.theta1), (t2, self.theta2), (l1, self.l1), (l2, self.l2), (l3, self.l3), (l4, self.l4), (l5, self.l5)])

    #     J_21 = diff(result7[1, 3], q1)
    #     J_21 = J_21.subs([(q1, Q1), (q2, Q2), (q3, Q3), (q4, Q4), (q5, Q5), (q6, Q6), (q7, Q7), (t1, self.theta1), (t2, self.theta2), (l1, self.l1), (l2, self.l2), (l3, self.l3), (l4, self.l4), (l5, self.l5)])
    #     J_22 = diff(result7[1, 3], q2)
    #     J_22 = J_22.subs([(q1, Q1), (q2, Q2), (q3, Q3), (q4, Q4), (q5, Q5), (q6, Q6), (q7, Q7), (t1, self.theta1), (t2, self.theta2), (l1, self.l1), (l2, self.l2), (l3, self.l3), (l4, self.l4), (l5, self.l5)])
    #     J_23 = diff(result7[1, 3], q3)
    #     J_23 = J_23.subs([(q1, Q1), (q2, Q2), (q3, Q3), (q4, Q4), (q5, Q5), (q6, Q6), (q7, Q7), (t1, self.theta1), (t2, self.theta2), (l1, self.l1), (l2, self.l2), (l3, self.l3), (l4, self.l4), (l5, self.l5)])
    #     J_24 = diff(result7[1, 3], q4)
    #     J_24 = J_24.subs([(q1, Q1), (q2, Q2), (q3, Q3), (q4, Q4), (q5, Q5), (q6, Q6), (q7, Q7), (t1, self.theta1), (t2, self.theta2), (l1, self.l1), (l2, self.l2), (l3, self.l3), (l4, self.l4), (l5, self.l5)])
    #     J_25 = diff(result7[1, 3], q5)
    #     J_25 = J_25.subs([(q1, Q1), (q2, Q2), (q3, Q3), (q4, Q4), (q5, Q5), (q6, Q6), (q7, Q7), (t1, self.theta1), (t2, self.theta2), (l1, self.l1), (l2, self.l2), (l3, self.l3), (l4, self.l4), (l5, self.l5)])
    #     J_26 = diff(result7[1, 3], q6)
    #     J_26 = J_26.subs([(q1, Q1), (q2, Q2), (q3, Q3), (q4, Q4), (q5, Q5), (q6, Q6), (q7, Q7), (t1, self.theta1), (t2, self.theta2), (l1, self.l1), (l2, self.l2), (l3, self.l3), (l4, self.l4), (l5, self.l5)])
    #     J_27 = diff(result7[1, 3], q7)
    #     J_27 = J_27.subs([(q1, Q1), (q2, Q2), (q3, Q3), (q4, Q4), (q5, Q5), (q6, Q6), (q7, Q7), (t1, self.theta1), (t2, self.theta2), (l1, self.l1), (l2, self.l2), (l3, self.l3), (l4, self.l4), (l5, self.l5)])

    #     J_31 = diff(result7[2, 3], q1)
    #     J_31 = J_31.subs([(q1, Q1), (q2, Q2), (q3, Q3), (q4, Q4), (q5, Q5), (q6, Q6), (q7, Q7), (t1, self.theta1), (t2, self.theta2), (l1, self.l1), (l2, self.l2), (l3, self.l3), (l4, self.l4), (l5, self.l5)])
    #     J_32 = diff(result7[2, 3], q2)
    #     J_32 = J_32.subs([(q1, Q1), (q2, Q2), (q3, Q3), (q4, Q4), (q5, Q5), (q6, Q6), (q7, Q7), (t1, self.theta1), (t2, self.theta2), (l1, self.l1), (l2, self.l2), (l3, self.l3), (l4, self.l4), (l5, self.l5)])
    #     J_33 = diff(result7[2, 3], q3)
    #     J_33 = J_33.subs([(q1, Q1), (q2, Q2), (q3, Q3), (q4, Q4), (q5, Q5), (q6, Q6), (q7, Q7), (t1, self.theta1), (t2, self.theta2), (l1, self.l1), (l2, self.l2), (l3, self.l3), (l4, self.l4), (l5, self.l5)])
    #     J_34 = diff(result7[2, 3], q4)
    #     J_34 = J_34.subs([(q1, Q1), (q2, Q2), (q3, Q3), (q4, Q4), (q5, Q5), (q6, Q6), (q7, Q7), (t1, self.theta1), (t2, self.theta2), (l1, self.l1), (l2, self.l2), (l3, self.l3), (l4, self.l4), (l5, self.l5)])
    #     J_35 = diff(result7[2, 3], q5)
    #     J_35 = J_35.subs([(q1, Q1), (q2, Q2), (q3, Q3), (q4, Q4), (q5, Q5), (q6, Q6), (q7, Q7), (t1, self.theta1), (t2, self.theta2), (l1, self.l1), (l2, self.l2), (l3, self.l3), (l4, self.l4), (l5, self.l5)])
    #     J_36 = diff(result7[2, 3], q6)
    #     J_36 = J_36.subs([(q1, Q1), (q2, Q2), (q3, Q3), (q4, Q4), (q5, Q5), (q6, Q6), (q7, Q7), (t1, self.theta1), (t2, self.theta2), (l1, self.l1), (l2, self.l2), (l3, self.l3), (l4, self.l4), (l5, self.l5)])
    #     J_37 = diff(result7[2, 3], q7)
    #     J_37 = J_37.subs([(q1, Q1), (q2, Q2), (q3, Q3), (q4, Q4), (q5, Q5), (q6, Q6), (q7, Q7), (t1, self.theta1), (t2, self.theta2), (l1, self.l1), (l2, self.l2), (l3, self.l3), (l4, self.l4), (l5, self.l5)])

    #     J = np.matrix([ [ J_11 , J_12 , J_13 , J_14 , J_15 , J_16 , J_17 ],\
    #                     [ J_21 , J_22 , J_23 , J_24 , J_25 , J_26 , J_27 ],\
    #                     [ J_31 , J_32 , J_33 , J_34 , J_35 , J_36 , J_37 ]], dtype='float')

    #     return J

    def compute_jacobian(self, r_joints_array):

        #Initializations
        l1  = self.l1
        l2  = self.l2
        l3  = self.l3
        l4  = self.l4
        l5  = self.l5

        th1 = self.theta1
        th2 = self.theta2

        x = l4 * math.sin(th1)
        y = l4 * math.cos(th1)
        z = l5 * math.sin(th2)
        w = l5 * math.cos(th2)

        q1 = r_joints_array[0]
        q2 = r_joints_array[1]
        q3 = r_joints_array[2]
        q4 = r_joints_array[3]
        q5 = r_joints_array[4]
        q6 = r_joints_array[5]
        q7 = r_joints_array[6]

        c1 = math.cos(q1)
        c2 = math.cos(q2)
        c3 = math.cos(q3)
        c4 = math.cos(q4)
        c5 = math.cos(q5)
        c6 = math.cos(q6)
        c7 = math.cos(q7)

        s1 = math.sin(q1)
        s2 = math.sin(q2)
        s3 = math.sin(q3)
        s4 = math.sin(q4)
        s5 = math.sin(q5)
        s6 = math.sin(q6)
        s7 = math.sin(q7)

        ########################

        J_11 = -l3*(c1*s3 + c2*c3*s1) - x*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - y*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - w*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - s5*(c1*c3 - c2*s1*s3))) - z*(s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - s5*(c1*c3 - c2*s1*s3))) - l2*s1*s2

        J_12 = x*(c1*c2*s4 - c1*c3*c4*s2) - z*(s6*(c1*c2*c4 + c1*c3*s2*s4) - c6*(c5*(c1*c2*s4 - c1*c3*c4*s2) - c1*s2*s3*s5)) - w*(c6*(c1*c2*c4 + c1*c3*s2*s4) + s6*(c5*(c1*c2*s4 - c1*c3*c4*s2) - c1*s2*s3*s5)) - y*(c1*c2*c4 + c1*c3*s2*s4) + l2*c1*c2 - l3*c1*c3*s2

        J_13 = w*(s6*(s5*(s1*s3 - c1*c2*c3) + c4*c5*(c3*s1 + c1*c2*s3)) - c6*s4*(c3*s1 + c1*c2*s3)) - l3*(c3*s1 + c1*c2*s3) - z*(c6*(s5*(s1*s3 - c1*c2*c3) + c4*c5*(c3*s1 + c1*c2*s3)) + s4*s6*(c3*s1 + c1*c2*s3)) - x*c4*(c3*s1 + c1*c2*s3) - y*s4*(c3*s1 + c1*c2*s3)

        J_14 = x*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - z*(s6*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2)) - w*(c6*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + c5*s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2)) - y*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4)

        J_15 = (z*c6 - w*s6)*(c3*c5*s1 + c1*c2*c5*s3 - c1*s2*s4*s5 + c4*s1*s3*s5 - c1*c2*c3*c4*s5)

        J_16 = w*(s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - s5*(c3*s1 + c1*c2*s3))) - z*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - s5*(c3*s1 + c1*c2*s3)))

        J_17 = 0

        ########################

        J_21 = l2*c1*s2 - x*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - y*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - w*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - s5*(c3*s1 + c1*c2*s3))) - z*(s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - s5*(c3*s1 + c1*c2*s3))) - l3*(s1*s3 - c1*c2*c3)

        J_22 = x*(c2*s1*s4 - c3*c4*s1*s2) - y*(c2*c4*s1 + c3*s1*s2*s4) - w*(c6*(c2*c4*s1 + c3*s1*s2*s4) + s6*(c5*(c2*s1*s4 - c3*c4*s1*s2) - s1*s2*s3*s5)) - z*(s6*(c2*c4*s1 + c3*s1*s2*s4) - c6*(c5*(c2*s1*s4 - c3*c4*s1*s2) - s1*s2*s3*s5)) + l2*c2*s1 - l3*c3*s1*s2

        J_23 = l3*(c1*c3 - c2*s1*s3) - w*(s6*(s5*(c1*s3 + c2*c3*s1) + c4*c5*(c1*c3 - c2*s1*s3)) - c6*s4*(c1*c3 - c2*s1*s3)) + z*(c6*(s5*(c1*s3 + c2*c3*s1) + c4*c5*(c1*c3 - c2*s1*s3)) + s4*s6*(c1*c3 - c2*s1*s3)) + x*c4*(c1*c3 - c2*s1*s3) + y*s4*(c1*c3 - c2*s1*s3)

        J_24 = w*(c6*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + c5*s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2)) + z*(s6*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2)) - x*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + y*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4)

        J_25 = -(z*c6 - w*s6)*(c1*c3*c5 - c2*c5*s1*s3 + c1*c4*s3*s5 + s1*s2*s4*s5 + c2*c3*c4*s1*s5)

        J_26 = z*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - s5*(c1*c3 - c2*s1*s3))) - w*(s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - s5*(c1*c3 - c2*s1*s3)))

        J_27 = 0

        ########################

        J_31 = 0

        J_32 = w*(s6*(c5*(s2*s4 + c2*c3*c4) + c2*s3*s5) + c6*(c4*s2 - c2*c3*s4)) - z*(c6*(c5*(s2*s4 + c2*c3*c4) + c2*s3*s5) - s6*(c4*s2 - c2*c3*s4)) - x*(s2*s4 + c2*c3*c4) + y*(c4*s2 - c2*c3*s4) - l2*s2 - l3*c2*c3

        J_33 = s2*(l3*s3 + x*c4*s3 + y*s3*s4 - z*c3*c6*s5 + w*c6*s3*s4 + w*c3*s5*s6 + z*s3*s4*s6 + z*c4*c5*c6*s3 - w*c4*c5*s3*s6)

        J_34 = x*(c2*c4 + c3*s2*s4) + y*(c2*s4 - c3*c4*s2) + w*(c6*(c2*s4 - c3*c4*s2) - c5*s6*(c2*c4 + c3*s2*s4)) + z*(s6*(c2*s4 - c3*c4*s2) + c5*c6*(c2*c4 + c3*s2*s4))

        J_35 = -(z*c6 - w*s6)*(c5*s2*s3 + c2*s4*s5 - c3*c4*s2*s5)

        J_36 = - w*(c6*(c5*(c2*s4 - c3*c4*s2) - s2*s3*s5) - s6*(c2*c4 + c3*s2*s4)) - z*(s6*(c5*(c2*s4 - c3*c4*s2) - s2*s3*s5) + c6*(c2*c4 + c3*s2*s4))
        
        J_37 = 0

        J = np.matrix([ [ J_11 , J_12 , J_13 , J_14 , J_15 , J_16 , J_17 ],\
                        [ J_21 , J_22 , J_23 , J_24 , J_25 , J_26 , J_27 ],\
                        [ J_31 , J_32 , J_33 , J_34 , J_35 , J_36 , J_37 ]])
        return J

    def tf_A01(self, r_joints_array):
        q1 = r_joints_array[0]
        c1 = math.cos(q1)
        s1 = math.sin(q1)

        tf = np.matrix([[c1 , -s1 , 0 , 0],\
                        [s1 , c1 , 0 , 0],\
                        [0 , 0 , 1 , self.l1],\
                        [0 , 0 , 0 , 1]])
        return tf

    def tf_A02(self, r_joints_array):
        q2 = r_joints_array[1]
        c2 = math.cos(q2)
        s2 = math.sin(q2)

        tf_A12 = np.matrix([[c2 , -s2 , 0 , 0],\
                            [0 , 0 , 1 , 0],\
                            [-s2 , -c2 , 0 , 0],\
                            [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A01(r_joints_array), tf_A12 )
        return tf

    def tf_A03(self, r_joints_array):
        q3 = r_joints_array[2]
        c3 = math.cos(q3)
        s3 = math.sin(q3)

        tf_A23 = np.matrix([[c3 , -s3 , 0 , 0],\
                            [0 , 0 , -1 , -self.l2],\
                            [s3 , c3 , 0 , 0],\
                            [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A02(r_joints_array), tf_A23 )
        return tf

    def tf_A04(self, r_joints_array):
        q4 = r_joints_array[3]
        c4 = math.cos(q4)
        s4 = math.sin(q4)

        tf_A34 = np.matrix([[c4 , -s4 , 0 , self.l3],\
                            [0 , 0 , -1 , 0],\
                            [s4 , c4 , 0 , 0],\
                            [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A03(r_joints_array), tf_A34 )
        return tf

    def tf_A05(self, r_joints_array):
        q5 = r_joints_array[4]
        c5 = math.cos(q5)
        s5 = math.sin(q5)

        tf_A45 = np.matrix([[c5 , -s5 , 0 , self.l4 * math.sin(self.theta1)],\
                            [0 , 0 , -1 , - self.l4 * math.cos(self.theta1)],\
                            [s5 , c5 , 0 , 0],\
                            [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A04(r_joints_array), tf_A45 )
        return tf

    def tf_A06(self, r_joints_array):
        q6 = r_joints_array[5]
        c6 = math.cos(q6)
        s6 = math.sin(q6)

        tf_A56 = np.matrix([[c6 , -s6 , 0 , 0],\
                            [0 , 0 , -1 , 0],\
                            [s6 , c6 , 0 , 0],\
                            [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A05(r_joints_array), tf_A56 )
        return tf

    def tf_A07(self, r_joints_array):
        q7 = r_joints_array[6]
        c7 = math.cos(q7)
        s7 = math.sin(q7)

        tf_A67 = np.matrix([[c7 , -s7 , 0 , self.l5 * math.sin(self.theta2)],\
                            [0 , 0 , 1 , self.l5 * math.cos(self.theta2)],\
                            [-s7 , -c7 , 0 , 0],\
                            [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A06(r_joints_array), tf_A67 )
        return tf
