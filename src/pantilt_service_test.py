#!/usr/bin/env python
import rospy
from dynamixel_workbench_msgs.srv import DynamixelCommand
from dynamixel_workbench_msgs.srv import DynamixelCommandRequest
from geometry_msgs.msg import Twist

#rosrun turtlesim turtle_teleop_key
servo_data1 = DynamixelCommandRequest()
servo_data2 = DynamixelCommandRequest()
pan_angle = 2000
tilt_angle = 2000
servo_data1.command = ""
servo_data2.command = ""
servo_data1.id = 1
servo_data2.id = 2
servo_data1.addr_name = "Goal_Position"
servo_data2.addr_name = "Goal_Position"
servo_data1.value = pan_angle
servo_data2.value = tilt_angle

def callback1(data):
    global servo_data1
    global servo_data2
    global pan_angle
    global tilt_angle
    pan_angle -= int(data.linear.x*10)
    tilt_angle += int(data.angular.z*10)
    servo_data1.value = pan_angle
    servo_data2.value = tilt_angle

if __name__ == "__main__":
    rospy.loginfo("test")
    rospy.init_node('pantilt_service_test')
    rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
    s_cliant = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
    sub = rospy.Subscriber('/turtle1/cmd_vel', Twist, callback1, queue_size=10)
    try:
        while not rospy.is_shutdown():
            result1 = s_cliant(servo_data1)
            result2 = s_cliant(servo_data2)
            rospy.loginfo("servo1:{}, servo2:{}".format(result1, result2))
            rospy.Rate(50).sleep()
    except KeyboardInterrupt:
        pass
