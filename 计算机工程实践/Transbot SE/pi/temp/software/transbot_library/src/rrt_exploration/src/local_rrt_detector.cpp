#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include "stdint.h"
#include "functions.h"
#include "mtrand.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_listener.h>

using namespace std;

// global variables
nav_msgs::OccupancyGrid mapData;
geometry_msgs::PointStamped clickedpoint;
geometry_msgs::PointStamped exploration_goal;
visualization_msgs::Marker points, line;
float xdim, ydim, resolution, Xstartx, Xstarty, init_map_x, init_map_y;

rdm r; // for genrating random numbers



//Subscribers callback functions---------------------------------------
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    mapData = *msg;
}


void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr &msg) {
    geometry_msgs::Point p;
    p.x = msg->point.x;
    p.y = msg->point.y;
    p.z = msg->point.z;
    points.points.push_back(p);

}


int main(int argc, char **argv) {
    unsigned long init[4] = {0x123, 0x234, 0x345, 0x456}, length = 7;
    MTRand_int32 irand(init, length); // 32-bit int generator
// this is an example of initializing by an array
// you may use MTRand(seed) with any 32bit integer
// as a seed for a simpler initialization
    MTRand drand; // double in [0, 1) generator, already init
// generate the same numbers as in the original C test program
    ros::init(argc, argv, "local_rrt_frontier_detector");
    ros::NodeHandle nh;
    // fetching all parameters
    float eta, init_map_x, init_map_y, range;
    std::string map_topic, base_frame_topic;
    std::string ns;
    ns = ros::this_node::getName();
    ros::param::param<float>(ns + "/eta", eta, 0.5);
    ros::param::param<std::string>(ns + "/map_topic", map_topic, "/robot_1/map");
    ros::param::param<std::string>(ns + "/robot_frame", base_frame_topic, "/robot_1/base_link");
//---------------------------------------------------------------
    ros::Subscriber sub = nh.subscribe(map_topic, 100, mapCallBack);
    ros::Subscriber rviz_sub = nh.subscribe("/clicked_point", 100, rvizCallBack);
    ros::Publisher targetspub = nh.advertise<geometry_msgs::PointStamped>("/detected_points", 10);
    ros::Publisher pub = nh.advertise<visualization_msgs::Marker>(ns + "_shapes", 10);
    ros::Rate rate(100);
// wait until map is received, when a map is received, mapData.header.seq will not be < 1  
//while (mapData.header.seq<1 or mapData.data.size()<1)  {  ros::spinOnce();  ros::Duration(0.1).sleep();}
//visualizations  points and lines..
    points.header.frame_id = "map";
    line.header.frame_id = "map";
    points.header.stamp = ros::Time(0);
    line.header.stamp = ros::Time(0);
    points.ns = line.ns = "markers";
    points.id = 0;
    line.id = 1;
    points.type = points.POINTS;
    line.type = line.LINE_LIST;
//Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    points.action = points.ADD;
    line.action = line.ADD;
    points.pose.orientation.w = 1.0;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.03;
    line.scale.y = 0.03;
    points.scale.x = 0.3;
    points.scale.y = 0.3;
    line.color.r = 255.0 / 255.0;
    line.color.g = 0.0 / 255.0;
    line.color.b = 0.0 / 255.0;
    points.color.r = 255.0 / 255.0;
    points.color.g = 0.0 / 255.0;
    points.color.b = 0.0 / 255.0;
    points.color.a = 0.3;
    line.color.a = 1.0;
    points.lifetime = ros::Duration();
    line.lifetime = ros::Duration();
    geometry_msgs::Point p;
    while (points.points.size() < 5) {
        ros::spinOnce();
        pub.publish(points);
    }
    std::vector<float> temp1;
    temp1.push_back(points.points[0].x);
    temp1.push_back(points.points[0].y);
    std::vector<float> temp2;
    temp2.push_back(points.points[2].x);
    temp2.push_back(points.points[0].y);
    init_map_x = Norm(temp1, temp2);
    temp1.clear();
    temp2.clear();
    temp1.push_back(points.points[0].x);
    temp1.push_back(points.points[0].y);
    temp2.push_back(points.points[0].x);
    temp2.push_back(points.points[2].y);
    init_map_y = Norm(temp1, temp2);
    temp1.clear();
    temp2.clear();
    Xstartx = (points.points[0].x + points.points[2].x) * .5;
    Xstarty = (points.points[0].y + points.points[2].y) * .5;
    geometry_msgs::Point trans;
    trans = points.points[4];
    std::vector<std::vector<float> > V;
    std::vector<float> xnew;
    xnew.push_back(trans.x);
    xnew.push_back(trans.y);
    V.push_back(xnew);
    points.points.clear();
    pub.publish(points);
    std::vector<float> frontiers;
    int i = 0;
    float xr, yr;
    std::vector<float> x_rand, x_nearest, x_new;
    tf::TransformListener listener;
// Main loop
    while (ros::ok()) {
// Sample free
        x_rand.clear();
        xr = (drand() * init_map_x) - (init_map_x * 0.5) + Xstartx;
        yr = (drand() * init_map_y) - (init_map_y * 0.5) + Xstarty;
        x_rand.push_back(xr);
        x_rand.push_back(yr);
// Nearest
        x_nearest = Nearest(V, x_rand);
// Steer
        x_new = Steer(x_nearest, x_rand, eta);
// ObstacleFree    1:free     -1:unkown (frontier region)      0:obstacle
        int checking = ObstacleFree(x_nearest, x_new, mapData);
        if (checking == -1) {
            exploration_goal.header.stamp = ros::Time(0);
            exploration_goal.header.frame_id = mapData.header.frame_id;
            exploration_goal.point.x = x_new[0];
            exploration_goal.point.y = x_new[1];
            exploration_goal.point.z = 0.0;
            p.x = x_new[0];
            p.y = x_new[1];
            p.z = 0.0;
            points.points.push_back(p);
            pub.publish(points);
//            cout << "local_rrt: " << exploration_goal << endl;
            targetspub.publish(exploration_goal);
            points.points.clear();
            V.clear();
            tf::StampedTransform transform;
            int temp = 0;
            while (temp == 0) {
                try {
                    temp = 1;
                    listener.lookupTransform(map_topic, base_frame_topic, ros::Time(0), transform);
                }
                catch (tf::TransformException ex) {
                    temp = 0;
                    ros::Duration(0.1).sleep();
                }
            }
            x_new[0] = transform.getOrigin().x();
            x_new[1] = transform.getOrigin().y();
            V.push_back(x_new);
            line.points.clear();
        } else if (checking == 1) {
            V.push_back(x_new);
            p.x = x_new[0];
            p.y = x_new[1];
            p.z = 0.0;
            line.points.push_back(p);
            p.x = x_nearest[0];
            p.y = x_nearest[1];
            p.z = 0.0;
            line.points.push_back(p);
        }
        pub.publish(line);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
