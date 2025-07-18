/**
 * 该例程将执行/turtle_vel_command服务，服务数据类型std_srvs/Trigger
 */
 
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>

ros::Publisher turtle_vel_pub;
bool pubvel = false;

// service回调函数，输入参数req，输出参数res
bool pubvelCallback(std_srvs::Trigger::Request  &req,
                    std_srvs::Trigger::Response &res)
{
    pubvel = !pubvel;

        ROS_INFO("Do you want to publish the vel?: [%s]", pubvel==true?"Yes":"No");// 打印客户端请求数据

    // 设置反馈数据
    res.success = true;
    res.message = "The status is changed!";

    return true;
}

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "turtle_vel_command_server");

  
    ros::NodeHandle n;

    // 创建一个名为/turtle_vel_command的server，注册回调函数pubvelCallback
    ros::ServiceServer command_service = n.advertiseService("/turtle_vel_command", pubvelCallback);

    // 创建一个Publisher，发布名为/turtle1/cmd_vel的topic，消息类型为geometry_msgs::Twist，队列长度8
    turtle_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 8);

    ros::Rate loop_rate(10);// 设置循环的频率

    while(ros::ok())
    {
        
        ros::spinOnce();// 查看一次回调函数队列
        
        // 判断pubvel为True，则发布小海龟速度指令
        if(pubvel)
        {
            geometry_msgs::Twist vel_msg;
            vel_msg.linear.x = 0.6;
            vel_msg.angular.z = 0.8;
            turtle_vel_pub.publish(vel_msg);
        }

        loop_rate.sleep();//按照循环频率延时
    }

    return 0;
}
