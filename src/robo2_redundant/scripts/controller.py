#!/usr/bin/env python3

"""
Start ROS node to publish angles for the position control of the xArm7.
"""

# Ros handlers services and messages
import rospy, roslib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
#Math imports
from math import sin, cos, atan2, pi, sqrt
from numpy.linalg import inv, det, norm, pinv
import numpy as np
import time as t
# import sympy as sm
from sympy import *

# Arm parameters
# xArm7 kinematics class
from kinematics import xArm7_kinematics

# from tf.transformations import quaternion_matrix
# matrix = quaternion_matrix([1, 0, 0, 0])

class xArm7_controller():
    """Class to compute and publish joints positions"""
    def __init__(self,rate):

        # Init xArm7 kinematics handler
        self.kinematics = xArm7_kinematics()

        # joints' angular positions
        self.joint_angpos = [0, 0, 0, 0, 0, 0, 0]
        # joints' angular velocities
        self.joint_angvel = [0, 0, 0, 0, 0, 0, 0]
        # joints' states
        self.joint_states = JointState()
        # joints' transformation matrix wrt the robot's base frame
        self.A01 = self.kinematics.tf_A01(self.joint_angpos)
        self.A02 = self.kinematics.tf_A02(self.joint_angpos)
        self.A03 = self.kinematics.tf_A03(self.joint_angpos)
        self.A04 = self.kinematics.tf_A04(self.joint_angpos)
        self.A05 = self.kinematics.tf_A05(self.joint_angpos)
        self.A06 = self.kinematics.tf_A06(self.joint_angpos)
        self.A07 = self.kinematics.tf_A07(self.joint_angpos)
        # gazebo model's states
        self.model_states = ModelStates()

        # ROS SETUP
        # initialize subscribers for reading encoders and publishers for performing position control in the joint-space
        # Robot
        self.joint_states_sub = rospy.Subscriber('/xarm/joint_states', JointState, self.joint_states_callback, queue_size=1)
        self.joint1_pos_pub = rospy.Publisher('/xarm/joint1_position_controller/command', Float64, queue_size=1)
        self.joint2_pos_pub = rospy.Publisher('/xarm/joint2_position_controller/command', Float64, queue_size=1)
        self.joint3_pos_pub = rospy.Publisher('/xarm/joint3_position_controller/command', Float64, queue_size=1)
        self.joint4_pos_pub = rospy.Publisher('/xarm/joint4_position_controller/command', Float64, queue_size=1)
        self.joint5_pos_pub = rospy.Publisher('/xarm/joint5_position_controller/command', Float64, queue_size=1)
        self.joint6_pos_pub = rospy.Publisher('/xarm/joint6_position_controller/command', Float64, queue_size=1)
        self.joint7_pos_pub = rospy.Publisher('/xarm/joint7_position_controller/command', Float64, queue_size=1)
        # Obstacles
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback, queue_size=1)

        # User created topics with queue size = rate
        self.pub1 = rospy.Publisher("/x_position_end_effector", Float64, queue_size=100)
        self.pub2 = rospy.Publisher("/y_position_end_effector", Float64, queue_size=100)
        self.pub3 = rospy.Publisher("/z_position_end_effector", Float64, queue_size=100)

        self.error_x = rospy.Publisher("/x_error_end_effector", Float64, queue_size=100)
        self.error_z = rospy.Publisher("/z_error_end_effector", Float64, queue_size=100)
        self.error_y = rospy.Publisher("/y_error_end_effector", Float64, queue_size=100)

        self.obs1 = rospy.Publisher("/y_position_Green_Obstacle", Float64, queue_size=100)
        self.obs2 = rospy.Publisher("/y_position_Red_Obstacle", Float64, queue_size=100)
        
        self.joint_3_dist_Green = rospy.Publisher("/Distance_joint_3_from_Green_Obstacle", Float64, queue_size=100)
        self.joint_3_dist_Red = rospy.Publisher("/Distance_joint_3_from_Red_Obstacle", Float64, queue_size=100)
        
        self.joint_4_dist_Green = rospy.Publisher("/Distance_joint_4_from_Green_Obstacle", Float64, queue_size=100)
        self.joint_4_dist_Red = rospy.Publisher("/Distance_joint_4_from_Red_Obstacle", Float64, queue_size=100)

        #Publishing rate
        self.period = 1.0/rate
        self.pub_rate = rospy.Rate(rate)

        self.publish()

    #SENSING CALLBACKS
    def joint_states_callback(self, msg):
        # ROS callback to get the joint_states

        self.joint_states = msg
        # (e.g. the angular position of joint 1 is stored in :: self.joint_states.position[0])

    def model_states_callback(self, msg):
        # ROS callback to get the gazebo's model_states

        self.model_states = msg
        # (e.g. #1 the position in y-axis of GREEN obstacle's center is stored in :: self.model_states.pose[1].position.y)
        # (e.g. #2 the position in y-axis of RED obstacle's center is stored in :: self.model_states.pose[2].position.y)


    def find_path(self, startingPosition, endingPosition, startingVelocity, endingVelocity, time):
        a0 = startingPosition
        a1 = startingVelocity
        a2 = (3 / time**2) * (endingPosition - startingPosition) - (2 / time) * startingVelocity - (1 / time) * endingVelocity
        a3 = - (2 / time**3) * (endingPosition - startingPosition) + (1 / time**2) * (startingVelocity + endingVelocity)

        return a0, a1, a2, a3

    def publish(self):

        # set configuration
        self.joint_angpos = [0, 0.75, 0, 1.5, 0, 0.75, 0]
        tmp_rate = rospy.Rate(1)
        tmp_rate.sleep()
        self.joint4_pos_pub.publish(self.joint_angpos[3])
        tmp_rate.sleep()
        self.joint2_pos_pub.publish(self.joint_angpos[1])
        self.joint6_pos_pub.publish(self.joint_angpos[5])
        tmp_rate.sleep()
        print("The system is ready to execute your algorithm...")

        rostime_now = rospy.get_rostime()
        time_now = rostime_now.to_nsec()

        # the parameters i desire for the first move from 0 -> B
        current_mode = 1
        prev_mode = 1
        moveTime = 2
        startTime = time_now / 1e9
        endTime = time_now / 1e9 + moveTime

        while not rospy.is_shutdown():

            # save the current time stamp
            currentTime = time_now / 1e9

            if (current_mode == 1):
            # move O -> B
                if (currentTime >= endTime):
                    current_mode = 2
                    continue

                a0, a1, a2, a3 = self.find_path(0, 0.2, 0, 0, moveTime)

            elif (current_mode == 2):
            # move B -> A
                moveTime = 4
                if (prev_mode != 2):
                    startTime = time_now / 1e9
                    endTime = time_now / 1e9 + moveTime
                    prev_mode = 2

                if (currentTime >= endTime):
                    current_mode = 3
                    continue

                a0, a1, a2, a3 = self.find_path(0.2, -0.2, 0, 0, moveTime)

            else:
            # move A -> B
                if (prev_mode != 3):
                    startTime = time_now / 1e9
                    endTime = time_now / 1e9 + moveTime
                    prev_mode = 3

                if (currentTime >= endTime):
                    current_mode = 2
                    continue

                a0, a1, a2, a3 = self.find_path(-0.2, 0.2, 0, 0, moveTime)
                
            ######## 1st task: follow trajectory

            # Find p1 to use for feedback control
            self.A07 = self.kinematics.tf_A07(self.joint_angpos)
            p1 = np.array(self.A07[0:3, 3])
            p1 = np.squeeze(np.asarray(p1))

            # Find desired position p1d
            y1d = a0 + a1 * (currentTime - startTime) + a2 * (currentTime - startTime)**2 + a3 * (currentTime - startTime)**3
            p1d = np.array([0.6043, y1d, 0.1508], dtype='float')

            # Find the desired velocities

            Y1d_deriv_number = a1 + 2 * a2 * (currentTime - startTime) + 3 * a3 * (currentTime - startTime)**2      
            p1d_deriv = np.array([0, Y1d_deriv_number, 0], dtype='float')
            
            # Compute jacobian matrix
            J_matrix = self.kinematics.compute_jacobian(self.joint_angpos)
            J = np.array(J_matrix)

            # pseudoinverse jacobian
            pinvJ = np.array(pinv(J_matrix))
            pinvJ = np.squeeze(np.asarray(pinvJ))

            # Compute the first task term of the velocities q' needed to be applied
            K1 = 1
            secondTerm = p1d_deriv + K1 * (p1d - p1)
            firstTask = np.dot(pinvJ, secondTerm)

            ######## 2nd task: obstacle avoidance

            Kc = 10
            K23 = 5
            K24 = 10
            self.A03 = self.kinematics.tf_A03(self.joint_angpos)
            self.A04 = self.kinematics.tf_A04(self.joint_angpos)

            # Middle of obstacles
            mid = (self.model_states.pose[1].position.y + self.model_states.pose[2].position.y) / 2
            
            if (current_mode == 2):
                mid = 0.08 + mid
            else:
                mid = -0.05 + mid

            # Distances from middle of the obstacles Criteria
            V1 = (1/2) * Kc * ((self.A03[1,3] - mid) ** 2)
            V2 = (1/2) * Kc * ((self.A04[1,3] - mid) ** 2)

            # Initializations
            l2 = self.kinematics.l2
            l3 = self.kinematics.l3
            l4 = self.kinematics.l4
            theta1 = self.kinematics.theta1

            # Angular positions
            q1 = self.joint_angpos[0]
            q2 = self.joint_angpos[1]
            q3 = self.joint_angpos[2]

            # Sine and cosine functions
            c1 = cos(q1)
            c2 = cos(q2)
            c3 = cos(q3)

            s1 = sin(q1)
            s2 = sin(q2)
            s3 = sin(q3)

            #Gradient of the criteria
            V1_grad = np.array([-Kc * (self.A03[1,3] - mid) * l2*c1*c2, -Kc * (self.A03[1,3] - mid)*l2*c2*s1, 0, 0, 0, 0, 0], dtype='float')
            V2_grad = np.array([-Kc * (self.A03[1,3] - mid) * (l2*c1*s2 - l3*(s1*s3 - c1*c2*c3)), -Kc * (self.A03[1,3] - mid) * (l2*c2*s1 - l3*c3*s1*s2), -Kc * (self.A03[1,3] - mid) * (l3*(c1*c3 - c2*s1*s3)), 0, 0, 0, 0], dtype='float')
            
            #Computing task 2

            In = np.eye(7)
            firstTerm = In - np.dot(pinvJ, J)
            qr2_deriv = K23 * V1_grad + K24 * V2_grad
            
            secondTask = np.dot(firstTerm, qr2_deriv)

            #Algorithm
            maximum = max(V1, V2)
            
            if (maximum >= 0.03):
                self.joint_angvel = firstTask + secondTask
            else:
                self.joint_angvel = firstTask

            # Convertion to angular position after integrating the angular speed in time
            # Calculate time interval
            time_prev = time_now
            rostime_now = rospy.get_rostime()
            time_now = rostime_now.to_nsec()
            dt = (time_now - time_prev)/1e9
            # Integration
            self.joint_angpos = np.add( self.joint_angpos, [index * dt for index in self.joint_angvel] )

            # Publish the new joint's angular positions
            self.joint1_pos_pub.publish(self.joint_angpos[0])
            self.joint2_pos_pub.publish(self.joint_angpos[1])
            self.joint3_pos_pub.publish(self.joint_angpos[2])
            self.joint4_pos_pub.publish(self.joint_angpos[3])
            self.joint5_pos_pub.publish(self.joint_angpos[4])
            self.joint6_pos_pub.publish(self.joint_angpos[5])
            self.joint7_pos_pub.publish(self.joint_angpos[6])

            self.pub1.publish(self.A07[0,3])
            self.pub2.publish(self.A07[1,3])
            self.pub3.publish(self.A07[2,3])

            self.error_x.publish(abs(self.A07[0,3] - 0.6043))
            self.error_z.publish(abs(self.A07[2,3] - 0.1508))
            self.error_y.publish(abs(self.A07[1,3] - y1d))

            green = self.model_states.pose[1].position.y + 0.05
            red = self.model_states.pose[2].position.y - 0.05
            
            self.obs1.publish(green)
            self.obs2.publish(red)

            self.joint_3_dist_Green.publish((self.A03[1,3] - 0.063) - green)
            self.joint_3_dist_Red.publish(red - (self.A03[1,3] + 0.063))

            self.joint_4_dist_Green.publish((self.A04[1,3] - 0.063) - green)
            self.joint_4_dist_Red.publish(red - (self.A03[1,3] + 0.063))

            self.pub_rate.sleep()

    def turn_off(self):
        pass

def controller_py():
    # Starts a new node
    rospy.init_node('controller_node', anonymous=True)
    # Reading parameters set in launch file
    rate = rospy.get_param("/rate")

    controller = xArm7_controller(rate)
    rospy.on_shutdown(controller.turn_off)
    rospy.spin()

if __name__ == '__main__':
    try:
        controller_py()
    except rospy.ROSInterruptException:
        pass
