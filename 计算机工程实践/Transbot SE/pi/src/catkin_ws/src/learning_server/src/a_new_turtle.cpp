/**
 * 该例程将请求小海龟节点里的/spawn服务，会在规定的位置出现一只新的小海龟
 */

#include <ros/ros.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "a_nes_turtle");// 初始化ROS节点

    ros::NodeHandle node;

    ros::service::waitForService("/spawn"); // 等待/spawn服务
    
    ros::ServiceClient new_turtle = node.serviceClient<turtlesim::Spawn>("/spawn");//创建一个服务客户端，连接名为/spawn的服务

    // 初始化turtlesim::Spawn的请求数据
    turtlesim::Spawn new_turtle_srv;
    new_turtle_srv.request.x = 6.0;
    new_turtle_srv.request.y = 8.0;
    new_turtle_srv.request.name = "turtle2";

    // 请求服务传入xy位置参数以及名字参数
    ROS_INFO("Call service to create a new turtle name is %s,at the x:%.1f,y:%.1f", new_turtle_srv.request.name.c_str(),
        new_turtle_srv.request.x, 
        new_turtle_srv.request.y);

    new_turtle.call(new_turtle_srv);

    
    ROS_INFO("Spwan turtle successfully [name:%s]", new_turtle_srv.response.name.c_str());// 显示服务调用结果

    return 0;
};

