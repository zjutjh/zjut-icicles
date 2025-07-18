
/*创建一个小海龟的速度发布者*/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
int main(int argc, char **argv){
    
    ros::init(argc, argv, "turtle_velocity_publisher");//ROS节点初始化
    
    ros::NodeHandle n;//这里是创建句柄

    //创建一个Publisher，发布名为/turtle1/cmd_vel的topic，消息类型为geometry_msgs::Twist，队列长度10
    ros::Publisher turtle_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    
    ros::Rate loop_rate(10);//设置循环的频率

    while (ros::ok()){
            //初始化需要发布的消息，类型需与Publisher一致
        geometry_msgs::Twist turtle_vel_msg;
        turtle_vel_msg.linear.x = 0.8;
        turtle_vel_msg.angular.z = 0.6;
        
        turtle_vel_pub.publish(turtle_vel_msg);// 发布速度消息

        //打印发布的速度内容
        ROS_INFO("Publsh turtle velocity command[%0.2f m/s, %0.2f rad/s]", turtle_vel_msg.linear.x, turtle_vel_msg.angular.z);
        
        loop_rate.sleep();//按照循环频率延时
    }
    return 0;
}

