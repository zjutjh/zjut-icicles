/*
 该例程将请求/Two_Int_Plus服务，服务数据类型learning_service::IntPlus
两个整型数相加求和
 */

#include <ros/ros.h>
#include "learning_server/IntPlus.h"
#include <iostream>
using namespace std;
int main(int argc, char** argv)
{
    int i,k;
    cin>>i;
    cin>>k;

    ros::init(argc, argv, "IntPlus_client");// 初始化ROS节点

    
    ros::NodeHandle node;// 创建节点句柄

    // 发现/Two_Int_Plus服务后，创建一个服务客户端
    ros::service::waitForService("/Two_Int_Plus");
    ros::ServiceClient IntPlus_client = node.serviceClient<learning_server::IntPlus>("/Two_Int_Plus");

    // 初始化learning_service::IntPlus的请求数据
    learning_server::IntPlus srv;
    srv.request.a = i;
    srv.request.b  = k;

    ROS_INFO("Call service to plus %d and %d", srv.request.a, srv.request.b);// 请求服务调用

    IntPlus_client.call(srv);

    // 显示服务调用结果
    ROS_INFO("Show the result : %d", srv.response.result);// 显示服务调用结果

    return 0;
}
