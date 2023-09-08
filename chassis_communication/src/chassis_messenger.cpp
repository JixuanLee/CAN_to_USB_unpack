
#include <tf/transform_broadcaster.h>
#include "chassis_communication/chassis_base_node.h"
#include <std_msgs/Bool.h>
#include "chassis_communication/chassis_message.h"
#include "chassis_communication/chassis_base_node.h"
#include "chassis_communication/protocol_process.h"
#include "chassis_communication/can_port.h"
#include <geometry_msgs/Twist.h>

  ProtocolParser protocolparser_;
  ChassisCanPort chassiscancort;

   void ChassisBaseNode::GetCanName(const std::string canname){
    can_ = std::make_shared<AsyncCAN>(canname); // 设置底层接口对象
  }

  void ChassisBaseNode::SetupSubPub(const std::string cmd_topic_name, 
                                                                                  const std::string brake_topic_name, 
                                                                                  const std::string chassis_heart_topic_name, 
                                                                                  const std::string chassis_status_topic_name, 
                                                                                  const std::string chassis_uwb_enable_topic_name)//ljx
  {

    heart_publisher_ = nh_->advertise<std_msgs::Bool>(chassis_heart_topic_name, 50);
    status_publisher_ = nh_->advertise<geometry_msgs::Twist>(chassis_status_topic_name, 10); //车辆底盘can上报的速度、车辆状态、控制模式等信息
    uwb_enable_publisher_ = nh_->advertise<std_msgs::Bool>(chassis_uwb_enable_topic_name, 10); //车辆底盘can上报的UWB使能状态

    brake_cmd_subscriber_ = nh_->subscribe(brake_topic_name, 5, &ChassisBaseNode::BrakeCmdCallback, this);
    motion_cmd_subscriber_ = nh_->subscribe<geometry_msgs::Twist>(cmd_topic_name, 5, &ChassisBaseNode::TwistCmdCallback, this);
  }

  // 判断是否下发刹车
  void ChassisBaseNode::BrakeCmdCallback( const std_msgs::Bool &msg)
  {
    brake_flag_twistcmdcallback == msg.data;
    
    // if(nerve_brake) //尚未急停过，正常接收数据
    // {
    //   brake_flag_twistcmdcallback = msg.data;
    //   if(brake_flag_twistcmdcallback == true) //如果是急停，此后不再接收刹车信号，一直保持急停状态，除非重启。这是为了防止诈尸。
    //   {
    //     nerve_brake = false;
    //   }
    // }
  }

  // 下发速度与刹车
  void ChassisBaseNode::TwistCmdCallback( const geometry_msgs::Twist::ConstPtr &msg)
  {
    if(brake_flag_twistcmdcallback == false)
    {
      ChassisBaseNode::SendMotionCommand(msg->linear.x, msg->angular.z, brake_flag_twistcmdcallback);
    }
    else
    {
      ChassisBaseNode::SendMotionCommand(0, 0, brake_flag_twistcmdcallback);
      ROS_WARN("接收到急停指令, 向CAN下发速度归零, 向下发急停信号");
    }
  }

 // 上报底盘至ROS 
  void ChassisBaseNode::PublishStateToROS()
  {
    current_time_ = ros::Time::now();
    double dt = (current_time_ - last_time_).toSec();

    static bool init_run = true;
    if (init_run)
    {
      last_time_ = current_time_;
      init_run = false;
      return;
    }
    
    //获取core_state_msgs_（刚刚CAN数据经过分类整理封包好的车辆上报信息）
    CoreStateMsgGroup core_state = chassiscancort.GetCoreStateToROS(); 

    // status_msg_to_ros status_msg_to_ros;
    // status_msg_to_ros.header.stamp = current_time_;
    // status_msg_to_ros.linear_velocity = core_state.motion_state.linear_velocity;// 线速度
    // status_msg_to_ros.angular_velocity = core_state.motion_state.angular_velocity;//角速度
    // status_publisher_.publish(status_msg_to_ros);

    geometry_msgs::Twist status_msg_to_ros;
    status_msg_to_ros.linear.x = core_state.motion_state.linear_velocity;
    status_msg_to_ros.angular.z = core_state.motion_state.angular_velocity;
    status_publisher_.publish(status_msg_to_ros);

    std_msgs::Bool uwb_enable;
    uwb_enable.data = false;
    uwb_enable.data = core_state.system_state.uwb_enable;
    uwb_enable_publisher_.publish(uwb_enable);

    // 发送心跳标志
    PublishHeartToROS(nerve_brake);

    // record time for next integration
    last_time_ = current_time_;
  }

  void ChassisBaseNode::SendMotionCommand(double linear, double angular, bool brake_flag_send)
  {
    if (can_ != nullptr && can_->IsOpened()) 
    {
      ChassisMessage msg;
      msg.type = ChassisMsgMotionCommand;
      msg.body.motion_command_msg.linear_velocity = linear;
      msg.body.motion_command_msg.angular_velocity = angular;
      msg.body.motion_command_msg.brake_flag = brake_flag_send;

      can_frame frame;
      if (protocolparser_.EncodeMessage(&msg, &frame)) 
      {
        ROS_INFO("Ready to send motion now.");
        can_->SendFrame(frame);
      }
      ROS_INFO("test_cmd_twist has send to CAN"); // 仅供测试
      send_flag = true;
    }
    else
    {
      ROS_ERROR("CAN port loss...Send motion cmd failed..");
      send_flag = false;
    }
  }

 // 上报心跳至ROS
  void ChassisBaseNode::PublishHeartToROS(bool nerve_brake)
  {
    std_msgs::Bool chassis_heart_flag;
    // if(nerve_brake && send_flag && receive_flag) // 尚未急停、且 发送、接收数据有效
    if(send_flag && receive_flag) // 尚未急停、且发送、接收数据有效
    {
      chassis_heart_flag.data = true;
    }
    else
    {
      chassis_heart_flag.data = false;
    }

    heart_publisher_.publish(chassis_heart_flag);
  }

