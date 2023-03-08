#!/usr/bin/env python
import rospy
from dynamixel_workbench_msgs.srv import DynamixelCommand
from dynamixel_workbench_msgs.srv import DynamixelCommandRequest
from geometry_msgs.msg import Point
from std_msgs.msg import Float32

def num2deg(num):
    n = num - 2048
    deg = n/1025
    return deg

def deg2num(deg):
    n = deg*1025
    num = n + 2048
    return num

def callback1(data):
    global servo_data1
    global pan_angle
    pan_angle.data -= data.Y
    servo_data1.value = deg2num(pan_angle.data)

if __name__ == "__main__":
    servo_data1 = DynamixelCommandRequest()
    pan_angle = Float32()
    pan_angle.data = 0.0
    servo_data1.command = ""
    servo_data1.id = 1
    servo_data1.addr_name = "Goal_Position"
    servo_data1.value = deg2num(pan_angle.data)
    rospy.init_node('pan_controller')
    rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
    s_cliant = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
    pub = rospy.Publisher("Pan_Degree", Float32, queue_size=10)
    sub = rospy.Subscriber("Detected_Object_Position", Point, callback1, queue_size=10)
    try:
        while not rospy.is_shutdown():
            result1 = s_cliant(servo_data1) #ここで待機が発生
            pub.publish(pan_angle)
            rospy.loginfo("servo1:{}".format(result1))
            rospy.Rate(5).sleep()
    except KeyboardInterrupt:
        pass