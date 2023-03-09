#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from dynamixel_workbench_msgs.srv import DynamixelCommand
from dynamixel_workbench_msgs.srv import DynamixelCommandRequest
from dynamixel_workbench_msgs.msg import DynamixelStateList
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Point
from std_msgs.msg import Float32

def num2deg(num):
    n = float(num - 2048)
    deg = n/1025*90
    return deg

def deg2num(deg):
    n = deg*1025/90
    num = int(n + 2048)
    return num


def sendGoalPos(pan_deg)
    
  trajectory_msgs::JointTrajectory jt_pt;

  jt_pt.header.frame_id = "pantilt_link";
  jt_pt.joint_names.resize(2);

  jt_pt.points.resize(1);
  jt_pt.points[0].positions.resize(2);
 
  jt_pt.joint_names[0] ="tilt";	  //チルトモータID:1
  jt_pt.joint_names[1] ="pan";		//パンモータID:2

  jt_pt.header.stamp = ros::Time::now();
  jt_pt.points[0].positions[0] = tilt_rad+tilt_com;   //チルトモータ目標角度
  jt_pt.points[0].positions[1] = pan_rad+pan_com;    //パンモータ目標角度
  jt_pt.points[0].time_from_start = ros::Duration(d_time);  //遷移時間
 
  pub_goal_pos.publish(jt_pt);

def callback1(data):
    global servo_data1
    global pan_angle
    #print(data.y)
    pan_angle.data = data.y/800 * 57 + num2deg(present_pos)
    servo_data1.value = deg2num(pan_angle.data)

def callback2(data):
    global present_pos
    present_pos = data.dynamixel_state[0].present_position

if __name__ == "__main__":
    servo_data1 = DynamixelCommandRequest()
    pan_angle = Float32()
    present_pos = 0
    pan_angle.data = 0.0
    servo_data1.command = ""
    servo_data1.id = 1
    servo_data1.addr_name = "Goal_Position"
    servo_data1.value = deg2num(pan_angle.data)
    rospy.init_node('pan_controller')
    rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
    s_cliant = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
    pub1 = rospy.Publisher("Pan_Degree", Float32, queue_size=10)
    pub2 = rospy.Publisher("/dynamixel_workbench/joint_trajectory", JointTrajectory, queue_size=1)
    sub1 = rospy.Subscriber("Detected_Object_Position", Point, callback1, queue_size=10)
    sub2 = rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, callback2, queue_size=10)
    pub1.publish(pan_angle)
    servo_data1.id = 2
    pub1.publish(pan_angle)
    servo_data1.id = 1

    try:
        while not rospy.is_shutdown():
            result1 = s_cliant(servo_data1) #ここで待機が発生
            pub1.publish(pan_angle)
            rospy.loginfo("servo1 : {}° {} {}".format(pan_angle.data, servo_data1.value, result1))
            rospy.Rate(50).sleep()
    except KeyboardInterrupt:
        pass


"""

void PanTiltController::sendGoalPos(double pan_rad, double tilt_rad, double d_time){
    
  trajectory_msgs::JointTrajectory jt_pt;

  jt_pt.header.frame_id = "pantilt_link";
  jt_pt.joint_names.resize(2);

  jt_pt.points.resize(1);
  jt_pt.points[0].positions.resize(2);
 
  jt_pt.joint_names[0] ="tilt";	  //チルトモータID:1
  jt_pt.joint_names[1] ="pan";		//パンモータID:2

  jt_pt.header.stamp = ros::Time::now();
  jt_pt.points[0].positions[0] = tilt_rad+tilt_com;   //チルトモータ目標角度
  jt_pt.points[0].positions[1] = pan_rad+pan_com;    //パンモータ目標角度
  jt_pt.points[0].time_from_start = ros::Duration(d_time);  //遷移時間
 
  pub_goal_pos.publish(jt_pt);


}
pub_goal_pos = nh_.advertise<trajectory_msgs::JointTrajectory>("/dynamixel_workbench/joint_trajectory",1);

rover_twist ni Twistgata wo nagereba ugocu
"""