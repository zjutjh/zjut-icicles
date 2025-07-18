#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg)
{
    
    static tf::TransformBroadcaster br;// 创建tf的广播器

    // 初始化tf数据
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );//设置xyz坐标
    tf::Quaternion q;
    q.setRPY(0, 0, msg->theta);//设置欧拉角：以x轴，y轴，z轴旋转
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));// 广播world与turtle坐标系之间的tf数据
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtle_world_tf_broadcaster");// 初始化ROS节点

    if (argc != 2)
    {
        ROS_ERROR("Missing a parameter as the name of the turtle!"); 
        return -1;
    }
    
    turtle_name = argv[1];// 输入参数作为海龟的名字

    // 订阅海龟的位姿话题/pose
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

        // 循环等待回调函数
    ros::spin();

    return 0;
};
