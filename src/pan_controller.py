#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from dynamixel_workbench_msgs.msg import DynamixelStateList
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

vertical_NoP = 800 #vertical number of pixels
vertical_AoV = 57 #vertical angle of view
horizontal_NoP = 1280
horizontal_AoV = 86
speed = 1 #

#def sendPanTiltPos():
#    global present_pos
    

def sendRoverTwist():
    rover_data = Twist()
    rover_data.linear.x = 0.0
    rover_data.linear.y = target_pos["pan"] * speed
    rover_data.angular.z = target_pos["tilt"]
    pub2.publish(rover_data)

def callback1(data):
    global target_pos
    target_pos["tilt"] = math.radians(data.x / horizontal_NoP * horizontal_AoV)
    target_pos["pan"] = math.radians(data.y / vertical_NoP * vertical_AoV)

def callback2(data):
    global present_pos
    present_pos["tilt"] = data.position[1]
    present_pos["pan"] = data.position[0]
    array = [0.0, 0.0]
    array[0] = present_pos["pan"] + target_pos["pan"]
    array[1] = present_pos["tilt"] + target_pos["tilt"]
    array_forpub = Float32MultiArray(data=array)
    pub1.publish(array_forpub)
    
if __name__ == "__main__":
    present_pos = {"pan":0.0, "tilt":0.0}
    target_pos = {"pan":0.0, "tilt":0.0}
    rospy.init_node('pan_controller')
    pub1 = rospy.Publisher("goal_pos", Float32MultiArray, queue_size=1)
    pub2 = rospy.Publisher("mouse_vel", Twist, queue_size=1)
    sub1 = rospy.Subscriber("Detected_Object_Position", Point, callback1, queue_size=10)
    sub2 = rospy.Subscriber("/dynamixel_workbench/joint_states", JointState, callback2, queue_size=10)
    try:
        while not rospy.is_shutdown():
            #sendPanTiltPos()
            sendRoverTwist()
            print("present:{}/{}\ntarget:{}/{}".format(present_pos["pan"]/3.14*180, present_pos["tilt"]/3.14*180, target_pos["pan"]/3.14*180, target_pos["tilt"]/3.14*180))
            rospy.Rate(50).sleep()
    except KeyboardInterrupt:
        pass
