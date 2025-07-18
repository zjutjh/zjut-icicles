//
// Created by yahboom on 2021/4/29.
//

#include <iostream>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <transbot_msgs/PointArray.h>
using namespace std;

class mediapipeCloud {
protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private;
    ros::Subscriber sub_point;
    ros::Publisher pub_pcl;
    pcl::visualization::CloudViewer viewer;
    ros::Timer viewer_timer;
    string cloud_topic;
    bool viewer_display= true;
public:
    mediapipeCloud(ros::NodeHandle &nh,ros::NodeHandle &nh_private):
    nh_(nh),
    nh_private(nh_private),
    viewer("Cloud Viewer") {
        nh_private.getParam("viewer_display", viewer_display);
        // 创建计时器 Create a timer
        viewer_timer = nh.createTimer(ros::Duration(0.1), &mediapipeCloud::timerClose, this);
        // 建立了一个点云发布者 Create a point cloud publisher
        pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("/mediapipe_cloud", 1000);
        // 创建订阅者 Create a subscriber
        sub_point = nh.subscribe("/mediapipe/points", 10, &mediapipeCloud::CallBackPoint, this);
    }
    void CallBackPoint(const transbot_msgs::PointArray &msg) {
        //点云大小为100 Point cloud size is 100
        unsigned int num_points = msg.points.size();
        //建立了一个pcl的点云（不能直接发布）
        // Create a PCL point cloud (cannot publish directly)
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        //点云初始化 Point cloud initialization
        cloud.points.resize(num_points);
        //建立一个可以直接发布的点云
        // Create a point cloud that can be published directly
        sensor_msgs::PointCloud2 output_msg;
        output_msg.header.stamp = ros::Time::now();
        for (int i = 0; i < msg.points.size(); i++) {
            geometry_msgs::Point Point;
            Point = msg.points[i];
            cloud.points[i].x = Point.x;
            cloud.points[i].y = Point.y;
            cloud.points[i].z = Point.z;
            cloud.points[i].r = 0;
            cloud.points[i].g = 255;
            cloud.points[i].b = 0;
        }
        //将点云转化为消息才能发布
        // Convert the point cloud to a message before publishing
        pcl::toROSMsg(cloud, output_msg);
        // Set the frame id to map
        output_msg.header.frame_id = "map";
        //发布出去 Publish
        pub_pcl.publish(output_msg);
//        ROS_INFO("Published ... ");
        if(viewer_display){
            // 使用PCL可视化工具显示出来 Use the PCL visualization tool to display it
            viewer.showCloud(cloud.makeShared());
//            ROS_INFO("showCloud ... ");
        }
    }
    void timerClose(const ros::TimerEvent &) {
        // 关闭可视化工具时，关闭ROS节点
        // Close the ROS node when closing the visualization tool
        if (viewer.wasStopped()) {
            ros::shutdown();
        }
    }
};


int main(int argc, char **argv) {
    //初始化了一个节点 Initializes a node
    ros::init(argc, argv, "mediapipe_Viewer");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    mediapipeCloud mediapipeCloud(nh,nh_private);
    ros::spin();
    return 0;
}
