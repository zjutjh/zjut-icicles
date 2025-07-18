/**
 * 该例程监听tf数据，并计算、发布turtle2的速度指令
   turtle2->turtle1 = world->turtle*world->turtle2
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "turtle1_turtle2_listener");// 初始化ROS节点

   
    ros::NodeHandle node; // 创建节点句柄

    // 请求服务产生turtle2
    ros::service::waitForService("/spawn");
    ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn srv;
    add_turtle.call(srv);

    // 创建发布turtle2速度控制指令的发布者
    ros::Publisher vel = node.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    tf::TransformListener listener;// 创建tf的监听器

    ros::Rate rate(10.0);

    while (node.ok())
    {
        // 获取turtle1与turtle2坐标系之间的tf数据
        tf::StampedTransform transform;
        try
        {
            listener.waitForTransform("/turtle2", "/turtle1", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("/turtle2", "/turtle1", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) 
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        // 根据turtle1与turtle2坐标系之间的位置关系，通过数学计算公式，得出角速度和线速度，发布turtle2的速度控制指令
        geometry_msgs::Twist turtle2_vel_msg;

        turtle2_vel_msg.angular.z = 6.0 * atan2(transform.getOrigin().y(),
                                        transform.getOrigin().x());
        turtle2_vel_msg.linear.x = 0.8 * sqrt(pow(transform.getOrigin().x(), 2) +
                                      pow(transform.getOrigin().y(), 2));
        vel.publish(turtle2_vel_msg);

        rate.sleep();
    }
    return 0;
};

