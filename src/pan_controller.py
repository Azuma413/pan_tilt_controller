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
import numpy as np

vertical_NoP = 800 #vertical number of pixels
vertical_AoV = 57 #vertical angle of view
horizontal_NoP = 1280
horizontal_AoV = 86
speed = 1 
pool_size = 100
target_pan_pool = np.zeros((pool_size))
target_tilt_pool = np.zeros((pool_size))
    
def sendRoverTwist():
    rover_data = Twist()
    rover_data.linear.x = 0.0
    rover_data.linear.y = target_pos["pan"] * speed
    rover_data.angular.z = target_pos["tilt"]
    pub2.publish(rover_data)

def callback1(data):
    global target_pos
    global target_pan_pool
    global target_tilt_pool
    target_pan_pool = np.append(target_pan_pool, data.y)
    target_tilt_pool = np.append(target_tilt_pool, data.x)
    target_pan_pool = np.delete(target_pan_pool, 0)
    target_tilt_pool = np.delete(target_tilt_pool, 0)
    target_pos["tilt"] = math.radians(np.mean(target_tilt_pool) / horizontal_NoP * horizontal_AoV)
    target_pos["pan"] = math.radians(np.mean(target_pan_pool) / vertical_NoP * vertical_AoV)

def callback2(data):
    global present_pos
    present_pos["tilt"] = data.position[1]
    present_pos["pan"] = data.position[0]
    array = [0.0, 0.0]
    array[0] = present_pos["pan"] + target_pos["pan"]
    array[1] = present_pos["tilt"] + target_pos["tilt"]
    #array[1] = 0.0
    array_forpub = Float32MultiArray(data=array)
    print("present:{}/{}\ntarget:{}/{}".format(present_pos["pan"]/3.14*180, present_pos["tilt"]/3.14*180, target_pos["pan"]/3.14*180, target_pos["tilt"]/3.14*180))
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
        #while not rospy.is_shutdown():
        #    #sendPanTiltPos()
        #    sendRoverTwist()
        #    rospy.Rate(50).sleep()
        rospy.spin()
    except KeyboardInterrupt:
        pass
