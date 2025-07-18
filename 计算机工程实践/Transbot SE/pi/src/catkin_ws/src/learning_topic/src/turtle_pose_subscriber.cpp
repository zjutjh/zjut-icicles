
/*创建一个小海龟的当前位姿信息接收*/
#include <ros/ros.h>
#include "turtlesim/Pose.h"
// 接收消息后，会进入消息回调函数，回调函数里边会对接收到的数据进行处理
void turtle_poseCallback(const turtlesim::Pose::ConstPtr& msg){
    // 打印接收到的消息
    ROS_INFO("Turtle pose: x:%0.3f, y:%0.3f", msg->x, msg->y);
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "turtle_pose_subscriber");// 初始化ROS节点

    ros::NodeHandle n;//这里是创建句柄

    // 创建一个订阅者，订阅的话题是/turtle1/pose的topic，poseCallback是回调函数
    ros::Subscriber pose_sub = n.subscribe("/turtle1/pose", 10, turtle_poseCallback);
   
    ros::spin(); // 循环等待回调函数

    return 0;
}
