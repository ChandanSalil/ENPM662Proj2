#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import sympy as sp
import numpy as np
import matplotlib.pyplot as plt

def manipulator_control(q1, q2, q3, q4, q5, q6):
    rospy.init_node('manipulator_control') 
    motor_1 = rospy.Publisher('/tbot1/robot_arm_shoulder_pan_control/command', Float64, queue_size=10)
    motor_2 = rospy.Publisher('/tbot1/robot_arm_shoulder_lift_control/command', Float64, 
queue_size=10)
    motor_3 = rospy.Publisher('/tbot1/robot_arm_elbow_control/command', Float64, queue_size=10)
    motor_4 = rospy.Publisher('/tbot1/robot_arm_wrist_control_1/command', Float64, queue_size=10)
    motor_5 = rospy.Publisher('/tbot1/robot_arm_wrist_control_2/command', Float64, queue_size=10)
    motor_6 = rospy.Publisher('/tbot1/robot_arm_wrist_control_3/command', Float64, queue_size=10)
    

    rate = rospy.Rate(4) 

    rospy.loginfo("Data is being sent")  
    rospy.loginfo(q1) 
    rospy.loginfo(q2) 
    rospy.loginfo(q3) 
    rospy.loginfo(q4) 
    rospy.loginfo(q5) 
    rospy.loginfo(q6) 
    rospy.loginfo("-----------------------") 

    # while not rospy.is_shutdown():
    twist = Float64()

    #Initial pose
    twist.data = q1
    motor_1.publish(twist)
    twist.data = q2
    motor_2.publish(twist)
    twist.data = q3
    motor_3.publish(twist)
    twist.data = q4
    motor_4.publish(twist)
    twist.data = q5
    motor_5.publish(twist)
    twist.data = q6
    motor_6.publish(twist)
    rate.sleep()

manipulator_control(-sp.pi/2, -sp.pi/2, 0,3*-sp.pi, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -sp.pi/2, 0,3*-sp.pi, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -sp.pi/2, 0,3*-sp.pi, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -sp.pi/2, 0,3*-sp.pi, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -sp.pi/2, 0,3*-sp.pi, sp.pi/2, 0)


manipulator_control(-sp.pi/2, -sp.pi/2, -sp.pi/2,3*-sp.pi, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -sp.pi/2, -sp.pi/2,3*-sp.pi, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -sp.pi/2, -sp.pi/2,3*-sp.pi, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -sp.pi/2, -sp.pi/2,3*-sp.pi, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -sp.pi/2, -sp.pi/2,3*-sp.pi, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -sp.pi/2, -sp.pi/2,3*-sp.pi, sp.pi/2, 0)

manipulator_control(-sp.pi/2, -3*sp.pi/4, -sp.pi/2,3*-sp.pi, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -3*sp.pi/4, -sp.pi/2,3*-sp.pi, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -3*sp.pi/4, -sp.pi/2,3*-sp.pi, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -3*sp.pi/4, -sp.pi/2,3*-sp.pi, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -3*sp.pi/4, -sp.pi/2,3*-sp.pi, sp.pi/2, 0)

manipulator_control(-sp.pi/2, -3*sp.pi/4, -sp.pi/2,-sp.pi/4, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -3*sp.pi/4, -sp.pi/2,-sp.pi/4, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -3*sp.pi/4, -sp.pi/2,-sp.pi/4, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -3*sp.pi/4, -sp.pi/2,-sp.pi/4, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -3*sp.pi/4, -sp.pi/2,-sp.pi/4, sp.pi/2, 0)






manipulator_control(-sp.pi/2, -3*sp.pi/4, -sp.pi/2,3*-sp.pi, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -3*sp.pi/4, -sp.pi/2,3*-sp.pi, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -3*sp.pi/4, -sp.pi/2,3*-sp.pi, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -3*sp.pi/4, -sp.pi/2,3*-sp.pi, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -3*sp.pi/4, -sp.pi/2,3*-sp.pi, sp.pi/2, 0)

manipulator_control(-sp.pi/2, -sp.pi/2, -sp.pi/2,3*-sp.pi, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -sp.pi/2, -sp.pi/2,3*-sp.pi, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -sp.pi/2, -sp.pi/2,3*-sp.pi, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -sp.pi/2, -sp.pi/2,3*-sp.pi, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -sp.pi/2, -sp.pi/2,3*-sp.pi, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -sp.pi/2, -sp.pi/2,3*-sp.pi, sp.pi/2, 0)

manipulator_control(-sp.pi/2, -sp.pi/2, 0,3*-sp.pi, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -sp.pi/2, 0,3*-sp.pi, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -sp.pi/2, 0,3*-sp.pi, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -sp.pi/2, 0,3*-sp.pi, sp.pi/2, 0)
manipulator_control(-sp.pi/2, -sp.pi/2, 0,3*-sp.pi, sp.pi/2, 0)


if __name__ == '__main__':
    try:
        0
    except rospy.ROSInterruptException: 
        pass
