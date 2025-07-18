/**
 * 该例程将订阅/company_info话题，自定义消息类型learning_topic::Information
 */
 
#include <ros/ros.h>
#include "learning_topic/Information.h"

// 接收到订阅的消息后，会进入消息回调函数处理数据
void CompanyInfoCallback(const learning_topic::Information::ConstPtr& msg)
{
    // 打印接收到的消息
    ROS_INFO("This is: %s  in %s", msg->company.c_str(), msg->city.c_str());
}

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "company_Information_subscriber");// 初始化ROS节点

    
    ros::NodeHandle n;// 这里是创建节点句柄

    // 创建一个Subscriber，订阅名话题/company_info的topic，注册回调函数CompanyInfoCallback
    ros::Subscriber person_info_sub = n.subscribe("/company_info", 10, CompanyInfoCallback);

    
    ros::spin();// 循环等待回调函数

    return 0;
}
