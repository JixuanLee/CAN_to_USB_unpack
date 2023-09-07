#include <memory>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "chassis_communication/protocol_process.h"
#include "chassis_communication/chassis_base_node.h"
#include "chassis_communication/can_port.h"

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "chassis_main_node");
  ros::NodeHandle private_node("~");
  setlocale(LC_ALL,"");

  ProtocolDetector detector;

  bool can_connect_flag = false;
  can_connect_flag = detector.IsConnect("can0"); //设置CAN帧接收回调函数（暂时无具体处理），并进行CAN连接反馈

  if(can_connect_flag == false)
  {
    ROS_ERROR("CAN硬件连接失败, ROS系统关闭。");
    ros::shutdown();
  }

  ChassisCanPort chassiscanport;
  ChassisBaseNode messenger;

  std::string port_name;
  private_node.param<std::string>("port_name", port_name, std::string("can0"));
  private_node.param<std::string>("cmd_topic_name", messenger.cmd_topic_name_,std::string("cmd_vel"));
  private_node.param<std::string>("brake_topic_name", messenger.brake_topic_name_, "brake");
  private_node.param<std::string>("chassis_heart_topic_name", messenger.chassis_heart_topic_name_, std::string("chassis"));                             
  private_node.param<std::string>("chassis_status_topic_name", messenger.chassis_status_topic_name_, std::string("chassis_status"));                             
  
  if (port_name.find("can") != std::string::npos) 
  {
    chassiscanport.Connect(port_name); //接收CAN上报数据并解包，按照协议分类处理数据，等待使用
    chassiscanport.EnableCommandedMode(); //根据协议向0x421下发CAN使能，允许通过CAN进行通讯控制接入
    ROS_INFO("已经接收CAN上报数据, 并下发CAN使能。");
  } 
  else 
  {
    ROS_ERROR("CAN通讯失败");
  }

  //当前：订阅到cmd速度信息后回调处理为can并下发can通信
  messenger.GetCanName(port_name);
  messenger.SetupSubPub(messenger.cmd_topic_name_, messenger.brake_topic_name_, messenger.chassis_heart_topic_name_,messenger.chassis_status_topic_name_); 

  ros::Rate rate(50);

  while (ros::ok()) 
  {
    messenger.PublishStateToROS(); // 处理从CAN上报、已经处理好的数据，分类发布ROS

    messenger.SendMotionCommand(0.01, 0, false); // WRAN:仅供测试

    ros::spinOnce();
    rate.sleep();

  }

  return 0;
}
