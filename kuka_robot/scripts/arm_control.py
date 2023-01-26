#!/usr/bin/env python3
# Authors: 
# <sparab> (Shantanu Suhas Parab)
# <vsingh03> (Vineet Kumar Singh)
# Description: ENPM662 F'22 Final Project Report

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Bool
import math
import sympy as sp
import numpy as np
from std_srvs.srv import Empty

class Arm:
    def __init__(self):
        self.joint_1 = rospy.Publisher('/kuka_robot/joint_1_controller/command', Float64, queue_size=10)
        self.joint_2 = rospy.Publisher('/kuka_robot/joint_2_controller/command', Float64, queue_size=10)
        self.joint_3 = rospy.Publisher('/kuka_robot/joint_3_controller/command', Float64, queue_size=10)
        self.joint_4 = rospy.Publisher('/kuka_robot/joint_4_controller/command', Float64, queue_size=10)
        self.joint_5 = rospy.Publisher('/kuka_robot/joint_5_controller/command', Float64, queue_size=10)
        self.joint_6 = rospy.Publisher('/kuka_robot/joint_6_controller/command', Float64, queue_size=10)
        self.grasp = rospy.Publisher('/kuka_robot/grasping', Bool, queue_size=10)
        self.joint_a1 = rospy.Publisher('/kuka_robot/joint_arm_1_controller/command', Float64, queue_size=10)
        self.joint_a2 = rospy.Publisher('/kuka_robot/joint_arm_2_controller/command', Float64, queue_size=10)
        self.joint_a3 = rospy.Publisher('/kuka_robot/joint_arm_3_controller/command', Float64, queue_size=10)
        self.joint_end_effector = rospy.Publisher('/kuka_robot/joint_end_effector_controller/command', Float64, queue_size=10)
        self.theta_1=0.0-math.radians(90)
        self.theta_2=0.0
        self.theta_3=0.0
        self.theta_4=0.0
        self.theta_5=0.0
        self.theta_6=0.0
        self.theta_ee=0.0
        self.theta_a1=0.0
        self.theta_a2=0.0
        self.theta_a3=0.0
        self.setup_variables()
        print("Variables Setup")
        self.calculating_jacobian()
        print("Computed Jacobian")
        self.compute_trajectory()
        print("Computed Trajectory")
        rospy.init_node('joint_value_publisher', anonymous=True)
        
    def setup_variables(self):
        # Defining Physical properties (link to link lengths) of the robot: 
        self.d1 = 0.32871
        self.d3 = 0.42090
        self.d5 = 0.37092
        self.d7 = 0.11345        
        self.iter=0
        self.n=100
        self.dt=20.0/self.n
        self.grasp_state=Bool()

        self.t_1=[math.radians(0)]    #Theta 1
        self.t_2=[math.radians(-40)]  #Theta 2
        self.t_4=[math.radians(100)]  #Theta 4
        self.t_5=[math.radians(0)]    #Theta 5
        self.t_6=[math.radians(30)]   #Theta 6
        self.t_7=[math.radians(0)]    #Theta 7

    def calculating_jacobian(self):
        theta_i, alpha_i, d_i, a_i, A_i, a_3, d_1, d_3, d_5, d_7 = sp.symbols('theta_i alpha_i d_i a_i A_i a_3 d_1, d_3, d_5, d_7')
        self.theta_1,self.theta_2,self.theta_3,self.theta_4,self.theta_5,self.theta_6,self.theta_7 = sp.symbols ('theta_1,theta_2, theta_3, theta_4, theta_5, theta_6, theta_7')
        # Storing DH parameters in arrays. /
        # Here a, alpha, d, theta denotes link length, link twist, link offset and joint angle respectively. 
        Rot_z = sp.Matrix([ [sp.cos(theta_i), -sp.sin(theta_i),0,0], [sp.sin(theta_i),sp.cos(theta_i),0,0], [0,0,1,0], [0,0,0,1] ])
        Rot_x = sp.Matrix([ [1,0,0,0], [0,sp.cos(alpha_i), -sp.sin(alpha_i),0], [0, sp.sin(alpha_i), sp.cos(alpha_i), 0], [0,0,0,1] ])
        Tran_z = sp.Matrix([[1,0,0,0], [0,1,0,0], [0,0,1,d_i], [0,0,0,1]])
        Tran_x = sp.Matrix([[1,0,0,a_i], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
        A_i=Rot_z*Tran_z*Tran_x*Rot_x

        # Calculating Rotation matrices for each link of robot
        A1=A_i.subs([(theta_i,self.theta_1-math.radians(90)),(alpha_i,math.radians(-90)),(a_i,0),(d_i,self.d1)])
        A2=A_i.subs([(theta_i,self.theta_2),(alpha_i,math.radians(90)),(a_i,0),(d_i,0)])
        A3=A_i.subs([(theta_i,math.radians(0)),(alpha_i,math.radians(90)),(a_i,0),(d_i,self.d3)])
        A4=A_i.subs([(theta_i,self.theta_4),(alpha_i,math.radians(-90)),(a_i,0),(d_i,0)])
        A5=A_i.subs([(theta_i,self.theta_5),(alpha_i,math.radians(-90)),(a_i,0),(d_i,self.d5)])
        A6=A_i.subs([(theta_i,self.theta_6),(alpha_i,math.radians(90)),(a_i,0),(d_i,0)])
        A7=A_i.subs([(theta_i,self.theta_7),(alpha_i,math.radians(0)),(a_i,0),(d_i,self.d7)])

        # Calculating transformation matrices for each link. 
        T1=A1
        T2=A1*A2
        # T3=A1*A2*A3
        T4=A1*A2*A3*A4
        T5=A1*A2*A3*A4*A5
        T6=A1*A2*A3*A4*A5*A6
        self.T7=A1*A2*A3*A4*A5*A6*A7

        # Calculating Z matrix from transforms. 
        Z0 = sp.Matrix([0,0,1])
        Z1 = T1[:3,2]
        Z2 = T2[:3,2]
        Z4 = T4[:3,2]
        Z5 = T5[:3,2]
        Z6 = T6[:3,2]
        Z7 = self.T7[:3,2]

        # Calculating Jacobian matrix using partial differentiation method. 
        Xp=self.T7[:3,3]
        diff_thet_1 = Xp.diff(self.theta_1) #Partially differentiating Xp wrt θ1

        diff_thet_2 = Xp.diff(self.theta_2) #Partially differentiating Xp wrt θ2

        diff_thet_4 = Xp.diff(self.theta_4) #Partially differentiating Xp wrt θ4

        diff_thet_5 = Xp.diff(self.theta_5) #Partially differentiating Xp wrt θ5

        diff_thet_6 = Xp.diff(self.theta_6) #Partially differentiating Xp wrt θ6

        diff_thet_7 = Xp.diff(self.theta_7) #Partially differentiating Xp wrt θ7
        
        self.J = sp.Matrix([[diff_thet_1[0],diff_thet_2[0],diff_thet_4[0],diff_thet_5[0],diff_thet_6[0],diff_thet_7[0]],
                [diff_thet_1[1],diff_thet_2[1],diff_thet_4[1],diff_thet_5[1],diff_thet_6[1],diff_thet_7[1]],
                [diff_thet_1[2],diff_thet_2[2],diff_thet_4[2],diff_thet_5[2],diff_thet_6[2],diff_thet_7[2]],
                [Z0[0],Z1[0],Z2[0],Z4[0],Z5[0],Z6[1]],[Z0[1],Z1[1],Z2[1],Z4[1],Z5[1],Z6[1]],[Z0[2],Z1[2],Z2[2],Z4[2],Z5[2],Z6[2]]])

    def compute_trajectory(self):
        x,y,z,r,o=sp.symbols("x y z r theta")
        T=self.T7
        x_tool=[]
        y_tool=[]
        z_tool=[]
        v=0.02
        X1=sp.Matrix([[0],[0],[v*2],[0],[0],[0]])
        X2=sp.Matrix([[0],[0],[v*2],[0],[0],[0]])
        X3=sp.Matrix([[-v*6],[-v*6],[0],[0],[0],[0]])
        X4=sp.Matrix([[0],[0],[-v*6],[0],[0],[0]])
        
        #Tool Velocity Matrix
        X_profile=[X1,X2,X3,X4]
        i=0
        print("Computing Trajectory")
        while i<=self.n:
            if(i<self.n/4):
                X_eval=X_profile[0]
            elif(i<self.n/2):
                X_eval=X_profile[1]
            elif(i<self.n*(3/4)):
                X_eval=X_profile[2]
            elif(i<self.n):
                X_eval=X_profile[3]
            
            T_eval=T.subs([(self.theta_1,self.t_1[i]),(self.theta_2,self.t_2[i]),(self.theta_4,self.t_4[i]),
                    (self.theta_5,self.t_5[i]),(self.theta_6,self.t_6[i]),(self.theta_7,self.t_7[i])])
            x_tool.append(T_eval[3])
            y_tool.append(T_eval[7])
            z_tool.append(T_eval[11])
            offset=0.0000001
            J_eval=self.J.subs([(self.theta_1,self.t_1[i]+offset),(self.theta_2,self.t_2[i]+offset),(self.theta_4,self.t_4[i]+offset),
                    (self.theta_5,self.t_5[i]+offset),(self.theta_6,self.t_6[i]+offset),(self.theta_7,self.t_7[i]+offset)])
            q=J_eval.pinv()*X_eval
            q=q*self.dt
            
            self.t_1.append(q[0]+self.t_1[i])
            self.t_2.append(q[1]+self.t_2[i])
            self.t_4.append(q[2]+self.t_4[i])
            self.t_5.append(q[3]+self.t_5[i])
            self.t_6.append(q[4]+self.t_6[i])
            self.t_7.append(q[5]+self.t_7[i])
            print(".",end="")
            i=i+1

    def gripper_close(self):
        self.theta_a1=0.01
        self.theta_a2=0.01
        self.theta_a3=0.01

    def gripper_open(self):
        self.theta_a1=-1.0
        self.theta_a2=-1.0
        self.theta_a3=-1.0

    # Add the code to compute joint angle here
    def compute_joint_angle(self):
        flag=False
        print(self.iter)
        if(self.iter>self.n):
            self.iter=0
        if(self.iter==0):
            print("Initial Pose")
            rospy.loginfo("Initial Pose")
            flag=True
        elif(self.iter==1):
            self.gripper_close()
            print("Close")
            rospy.loginfo("Close")
            flag=True
        elif(self.iter==self.n):
            self.gripper_open()
            print("Open")
            rospy.loginfo("Open")
            flag=True
        
        self.theta_1=self.t_1[self.iter]
        self.theta_2=self.t_2[self.iter]
        self.theta_3=0.0
        self.theta_4=self.t_4[self.iter]
        self.theta_5=self.t_5[self.iter]
        self.theta_6=self.t_6[self.iter]
        self.theta_7=self.t_7[self.iter]
        self.iter=self.iter+1
        return flag
            
    # Do not touch this part its the publisher
    def controller(self):    
        rate = rospy.Rate(2) # 10hz
        while not rospy.is_shutdown():
            f=self.compute_joint_angle()
            hello_str = "arm 1 %s" % self.theta_a1
            hello_str1= "arm 2 %s" % self.theta_a2
            hello_str2= "arm 3 %s" % self.theta_a3
            hello_str3 = "Theta 4 %s" % self.theta_4
            hello_str4 = "Theta 5 %s" % self.theta_5
            rospy.loginfo(hello_str)
            rospy.loginfo(hello_str1)
            rospy.loginfo(hello_str2)
            rospy.loginfo(hello_str3)
            rospy.loginfo(hello_str4)
            self.joint_1.publish(self.theta_1)
            self.joint_2.publish(self.theta_2)
            self.joint_3.publish(self.theta_3)
            self.joint_4.publish(self.theta_4)
            self.joint_5.publish(self.theta_5)
            self.joint_6.publish(self.theta_6)
            self.grasp.publish(self.grasp_state)
            self.joint_end_effector.publish(self.theta_ee)
            self.joint_a1.publish(self.theta_a1)
            self.joint_a2.publish(self.theta_a2)
            self.joint_a3.publish(self.theta_a3)
            if f:
                rospy.sleep(10)
            rate.sleep()

if __name__ == '__main__':
    try:
        a= Arm()
        a.controller()
    except rospy.ROSInterruptException:
        pass