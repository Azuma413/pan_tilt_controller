#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from dynamixel_workbench_msgs.msg import DynamixelStateList
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

vertical_NoP = 800 #vertical number of pixels
vertical_AoV = 57 #vertical angle of view
horizontal_NoP = 1280
horizontal_AoV = 86
speed = 1 #

def sendPanTiltPos():
    d_time = 0.02
    jt_pt = JointTrajectory()
    jt_pt.header.frame_id = "pantilt_link"
    jt_pt.joint_names[0] ="tilt"	  #チルトモータID:1
    jt_pt.joint_names[1] ="pan"		#パンモータID:2
    jt_pt.header.stamp = rospy.Time.now()
    #jt_pt.points[0].positions[0] = present_pos["tilt"] + target_pos["tilt"]  #チルトモータ目標角度
    jt_pt.points[0].positions[0] = 0                                          #縦方向を固定
    jt_pt.points[0].positions[1] = present_pos["pan"] + target_pos["pan"]     #パンモータ目標角度
    jt_pt.points[0].time_from_start = rospy.Duration(d_time)  #遷移時間
    pub1.publish(jt_pt)

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
    present_pos["tilt"] = data.position[0]
    present_pos["pan"] = data.position[1]
    
if __name__ == "__main__":
    present_pos = {"pan":0.0, "tilt":0.0}
    target_pos = {"pan":0.0, "tilt":0.0}
    rospy.init_node('pan_controller')
    pub1 = rospy.Publisher("/dynamixel_workbench/joint_trajectory", JointTrajectory, queue_size=1)
    pub2 = rospy.Publisher("mouse_vel", Twist, queue_size=1)
    sub1 = rospy.Subscriber("Detected_Object_Position", Point, callback1, queue_size=10)
    sub2 = rospy.Subscriber("/joint_states", JointState, callback2, queue_size=10)
    try:
        while not rospy.is_shutdown():
            sendPanTiltPos()
            sendRoverTwist()
            print("present:{}/{}\ntarget:{}/{}".format(present_pos["pan"], present_pos["tilt"], target_pos["pan"], target_pos["tilt"]))
            rospy.Rate(50).sleep()
    except KeyboardInterrupt:
        pass
