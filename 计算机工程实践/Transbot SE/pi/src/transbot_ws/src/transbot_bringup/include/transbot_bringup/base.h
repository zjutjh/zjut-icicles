#ifndef RIKI_BASE_H
#define RIKI_BASE_H
#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <string>
class RobotBase {
public:
    RobotBase();

    void velCallback(const geometry_msgs::Twist twist);

private:
    ros::NodeHandle nh_;
    ros::Publisher odom_publisher_;
    ros::Subscriber velocity_subscriber_;
    ros::Time last_vel_time_;
    tf::TransformBroadcaster odom_broadcaster_;
    float linear_scale_;
    float linear_velocity_x_;
    float linear_velocity_y_;
    float angular_velocity_z_;
    float vel_dt_;
    float x_pos_;
    float y_pos_;
    float heading_;
    std::string namespace_;
    bool is_multi_robot_;
};

#endif
