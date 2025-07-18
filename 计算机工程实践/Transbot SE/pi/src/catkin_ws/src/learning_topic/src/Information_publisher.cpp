
/**
 * 该例程将发布/company_info话题，消息类型是自定义的learning_topic::Information
 */
 
#include <ros/ros.h>
#include "learning_topic/Information.h"

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "company_Information_publisher");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Publisher，发布名为/company_info的topic，消息类型为learning_topic::Person，队列长度10
    ros::Publisher Information_pub = n.advertise<learning_topic::Information>("/company_info", 10);

    // 设置循环的频率
    ros::Rate loop_rate(1);

    int count = 0;
    while (ros::ok())
    {
        // 初始化learning_topic::Information类型的消息
        learning_topic::Information info_msg;
        info_msg.company = "Yahboom";
        info_msg.city = "Shenzhen";

        // 发布消息
        Information_pub.publish(info_msg);

        ROS_INFO("Information: company:%s  city:%s ", 
                  info_msg.company.c_str(), info_msg.city.c_str());

        
        loop_rate.sleep();// 按照循环频率延时
    }

    return 0;
}

