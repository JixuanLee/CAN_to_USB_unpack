#ifndef CHASSIS_BASE_NODE_H
#define CHASSIS_BASE_NODE_H

#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <mutex>
#include <std_msgs/Bool.h>
#include "async_can.hpp"

class ChassisBaseNode
{
public:

    std::string odom_frame_;
    std::string base_frame_;
    std::string cmd_topic_name_; 
    std::string brake_topic_name_; 
    std::string chassis_heart_topic_name_;
    std::string chassis_status_topic_name_;

    bool brake_flag_twistcmdcallback = false;
    bool nerve_brake = true;
    bool send_flag = true;
    bool receive_flag = true; //暂未使用到

    void SetupSubPub(std::string cmd_topic_name, std::string brake_topic_name, std::string chassis_heart_topic_name, const std::string chassis_status_topic_name);
    void PublishStateToROS();
    void GetCanName(const std::string canname);
    void SendMotionCommand(double linear, double angular, bool brake_flag_send);
    void PublishHeartToROS( bool n_brake);

private:

    ros::NodeHandle *nh_;
    std::shared_ptr<AsyncCAN> can_;

    std::mutex twist_mutex_;
    geometry_msgs::Twist current_twist_;

    ros::Publisher heart_publisher_;
    ros::Publisher status_publisher_;
    ros::Subscriber motion_cmd_subscriber_;
    ros::Subscriber brake_cmd_subscriber_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    double linear_speed_ = 0.0;
    double angular_speed_ = 0.0;
    double position_x_ = 0.0;
    double position_y_ = 0.0;
    double theta_ = 0.0;

    ros::Time last_time_;
    ros::Time current_time_;

    
    void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void BrakeCmdCallback(const std_msgs::Bool &msg);
};


#endif /* CHASSIS_BASE_NODE_H */
