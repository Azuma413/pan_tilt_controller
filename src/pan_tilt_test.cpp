#include <ros/ros.h>
#include <ros/time.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float32MultiArray.h>

#include <string>
using namespace std;

/*********************************
 * プロトタイプ宣言
 */


/*********************************
 * パンチルト装置に目標姿勢を送信し、現在位置,速度,負荷を取得するクラス
 */
double pan_rad = 0.0;		//パン方向の目標角度[rad]
double tilt_rad = 0.0;	//チルト方向の目標角度[rad]

class PanTiltController{
  public:
  PanTiltController();

  void panTiltInit();
  void sendGoalPos(double send_pan_rad, double send_tilt_rad, double d_time);

  double getPosition(string name);
  double getVelocity(string name);
  double getEffort(string name);

  private:
  ros::NodeHandle nh_;
  ros::Publisher  pub_goal_pos;
  ros::Subscriber sub_status;
  ros::Subscriber sub_goal_pos;
  ros::ServiceClient dynamixel_req;

  sensor_msgs::JointState pan_tilt_state;

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void subGoalCallback(const std_msgs::Float32MultiArray& data);
};

PanTiltController::PanTiltController(){

  pub_goal_pos = nh_.advertise<trajectory_msgs::JointTrajectory>("/dynamixel_workbench/joint_trajectory",1);
  sub_status   = nh_.subscribe("/dynamixel_workbench/joint_states", 1, &PanTiltController::jointStateCallback, this);
  sub_goal_pos = nh_.subscribe("goal_pos", 1, &PanTiltController::subGoalCallback, this);
  dynamixel_req = nh_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");

  pan_tilt_state.name.resize(2);
  pan_tilt_state.position.resize(2);
  pan_tilt_state.velocity.resize(2);
  pan_tilt_state.effort.resize(2);

    
}

void PanTiltController::panTiltInit(){

  dynamixel_workbench_msgs::DynamixelCommand com;

  com.request.command   = "";
  com.request.id        = 1;
  com.request.addr_name = "Profile_Velocity";
  com.request.value     = 20;

  while(!dynamixel_req.call(com)){
    ROS_ERROR("Failed to call service dynamixel_req");
  }
  ROS_INFO("SET DATA: id: %d, addr_name: %s, value: %d", com.request.id, com.request.addr_name.c_str(), com.request.value);

  com.request.id        = 2;
  com.request.addr_name = "Profile_Velocity";
  com.request.value     = 20;

  while(!dynamixel_req.call(com)){
    ROS_ERROR("Failed to call service dynamixel_req");
  }
  ROS_INFO("SET DATA: id: %d, addr_name: %s, value: %d", com.request.id, com.request.addr_name.c_str(), com.request.value);

  com.request.id        = 1;
  com.request.addr_name = "Position_P_Gain";
  com.request.value     = 4000;

  while(!dynamixel_req.call(com)){
    ROS_ERROR("Failed to call service dynamixel_req");
  }
  ROS_INFO("SET DATA: id: %d, addr_name: %s, value: %d", com.request.id, com.request.addr_name.c_str(), com.request.value);

  com.request.id        = 2;
  com.request.addr_name = "Position_P_Gain";
  com.request.value     = 2000;

  while(!dynamixel_req.call(com)){
    ROS_ERROR("Failed to call service dynamixel_req");
  }
  ROS_INFO("SET DATA: id: %d, addr_name: %s, value: %d", com.request.id, com.request.addr_name.c_str(), com.request.value);

  com.request.id        = 1;
  com.request.addr_name = "Position_I_Gain";
  com.request.value     = 400;

  while(!dynamixel_req.call(com)){
    ROS_ERROR("Failed to call service dynamixel_req");
  }
  ROS_INFO("SET DATA: id: %d, addr_name: %s, value: %d", com.request.id, com.request.addr_name.c_str(), com.request.value);

  com.request.id        = 2;
  com.request.addr_name = "Position_I_Gain";
  com.request.value     = 400;

  while(!dynamixel_req.call(com)){
    ROS_ERROR("Failed to call service dynamixel_req");
  }
  ROS_INFO("SET DATA: id: %d, addr_name: %s, value: %d", com.request.id, com.request.addr_name.c_str(), com.request.value);


  
}


void PanTiltController::sendGoalPos(double send_pan_rad, double send_tilt_rad, double d_time){
    
  trajectory_msgs::JointTrajectory jt_pt;

  jt_pt.header.frame_id = "pantilt_link";
  jt_pt.joint_names.resize(2);

  jt_pt.points.resize(1);
  jt_pt.points[0].positions.resize(2);
 
  jt_pt.joint_names[0] ="tilt";	  //チルトモータID:1
  jt_pt.joint_names[1] ="pan";		//パンモータID:2

  jt_pt.header.stamp = ros::Time::now();
  jt_pt.points[0].positions[0] = send_tilt_rad;   //チルトモータ目標角度
  jt_pt.points[0].positions[1] = send_pan_rad;    //パンモータ目標角度
  jt_pt.points[0].time_from_start = ros::Duration(d_time);  //遷移時間
 
  pub_goal_pos.publish(jt_pt);


}


void PanTiltController::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){

  pan_tilt_state.header.stamp = msg->header.stamp;
  pan_tilt_state.name[0] = msg->name[0];
  pan_tilt_state.name[1] = msg->name[1];
  pan_tilt_state.position[0] = msg->position[0];
  pan_tilt_state.position[1] = msg->position[1];
  pan_tilt_state.velocity[0] = msg->velocity[0];
  pan_tilt_state.velocity[1] = msg->velocity[1];
  pan_tilt_state.effort[0] = msg->effort[0];
  pan_tilt_state.effort[1] = msg->effort[1];

}

double PanTiltController::getPosition(string name){

  int i;
  for(i = 0; i < 2; i++){
    if(pan_tilt_state.name[i] == name){
      return pan_tilt_state.position[i];
    }    
  }

  return 0;

}

double PanTiltController::getVelocity(string name){

  int i;
  for(i = 0; i < 2; i++){
    if(pan_tilt_state.name[i] == name){
      return pan_tilt_state.velocity[i];
    }    
  }

  return 0;

}

double PanTiltController::getEffort(string name){

  int i;
  for(i = 0; i < 2; i++){
    if(pan_tilt_state.name[i] == name){
      return pan_tilt_state.effort[i];
    }    
  }

  return 0;

}
void PanTiltController::subGoalCallback(const std_msgs::Float32MultiArray& data){
  pan_rad = data.data[0];
  tilt_rad = data.data[1]; 
}

/*********************************
 * メイン
 */
int main(int argc, char **argv){

  ros::init(argc, argv, "pan_tilt_test");
  PanTiltController pan_tilt_controller;
  ros::NodeHandle nh;
  ros::Rate loop_rate(40);
  pan_tilt_controller.panTiltInit();
  double delta_rad = 0.004;	//1周期ごとの角度変化量[rad]
  double send_tilt_rad = 0.0;
  double send_pan_rad = 0.0;

  while(nh.ok()){
/*
    if(send_pan_rad >= 0.5 && delta_rad > 0.0){
      delta_rad = -delta_rad;
    }else if(send_pan_rad <= -0.5 && delta_rad < 0.0){
      delta_rad = -delta_rad;
    }
    */
   if (pan_rad > send_pan_rad){
    send_pan_rad += delta_rad;
   }else{
    send_pan_rad -= delta_rad;
   }
   if (tilt_rad > send_tilt_rad){
    send_tilt_rad += delta_rad;
   }else{
    send_tilt_rad -= delta_rad;
   }

    pan_tilt_controller.sendGoalPos(send_pan_rad, send_tilt_rad, 0.02);
    ros::spinOnce();
    loop_rate.sleep();

    ROS_INFO("pan_pos: %f || tilt_pos: %f", pan_tilt_controller.getPosition("pan"), pan_tilt_controller.getPosition("tilt"));
  }
}
