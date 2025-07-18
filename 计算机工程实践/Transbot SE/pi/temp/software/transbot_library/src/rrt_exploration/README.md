# rrt_exploration
It is a ROS package that implements a multi-robot map exploration algorithm for mobile robots. It is based on the Rapidly-Exploring Random Tree (RRT) algorithm. It uses occupancy girds as a map representation.The packgae has 5 different ROS nodes:

  - Global RRT frontier point detector node.
  
  - Local RRT frontier point detector node.
  
  - Filter node.
  
  - Assigner node.
  
  - opencv-based frontier detector node.

This is a [Youtube playlist](https://www.youtube.com/playlist?list=PLoGH52eUIHsc1B_xPLL6ogzYxrWy675kr) showing the package running on single/multiple robots, using real setup (Kobuki robots) and simulation (Gazebo).


Note: This package was written during my master's thesis at the American University of Sharjah. My thesis advisor is Dr. [Shayok Mukhopadhyay](https://sites.google.com/site/shayok/Home). If you are using this package in your research work, please cite this [paper](http://ieeexplore.ieee.org/document/8202319/).

## 1. Requirements
The package has been tested on both ROS Kinetic and ROS Indigo, it should work on other distributions like Jade. The following requirements are needed before installing the package:

1- You should have installed a ROS distribution (indigo or later. Recommended is either indigo or kinetic).

2- Created a workspace.

3- Installed the "gmapping" ROS package: on Ubuntu, and if you are running ROS Kinectic, you can do that by typing the following command in the terminal:

```sh
$ sudo apt-get install ros-kinetic-gmapping
```
4- Install ROS navigation stack. You can do that with the following command (assuming Ubuntu, ROS Kinetic):
```sh
$ sudo apt-get install ros-kinetic-navigation
```
5- You should have Python 2.7. (it was not tested on Python 3).

6-You should have/install the following python modules:

-OpenCV (cv2)
```sh
$ sudo apt-get install python-opencv
```
-Numpy
```sh
$ sudo apt-get install python-numpy
```
-Sklearn
```sh
$ sudo apt-get install python-scikits-learn
```
## 2. Installation
Download the package and place it inside the ```/src``` folder in your workspace. And then compile using ```catkin_make```.
## 3. Setting Up Your Robots
This package provides an exploration strategy for single or multiple robots. However, for it to work, you should have set your robots ready using the [navigation stack](http://wiki.ros.org/navigation). Additionally, the robots must be set and prepared as follows.

Note: If you want to quickly run and test the package, you can try out the [rrt_exploration_tutorials](https://github.com/hasauino/rrt_exploration_tutorials) package which provides Gazebo simulation for single and multiple robots, you can use it to directly with this package.

### 3.1. Robots Network
For the multi-robot configuration, the package doesn't require special network configuration, it simply works by having a single ROS master (can be one of the robots). So on the other robots, the ```ROS_MASTER_URI``` parameter should be pointing at the master's address. 
For more information on setting up ROS on multiple machines, follow [this](http://wiki.ros.org/ROS/NetworkSetup) link.

### 3.2. Robot's frame names in ```tf```
All robot's frames should be prefixed by its name. Naming of robots starts from "/robot_1", "/robot_2", "/robot_3", .. and so on. Even if you are using the package for single robot, robot's frames should be prefixed by its name (i.e. /robot_1). So for robot_1, the frames in the ```tf``` tree should look like this:

![alt text](https://github.com/hasauino/storage/blob/master/pictures/framesTf.png "robot_1 frames")

### 3.3. Robot's node and topic names
All the nodes and topics running on a robot must also be prefixed by its name. For robot 1, node names should look like: ```/robot_1/move_base_node```,  ```/robot_1/slam_gmapping```.

And topic names should be like: ```/robot_1/odom```,  ```/robot_1/map```,  ```/robot_1/base_scan```, ..etc.

### 3.4. Setting up the navigation stack on the robots
The ```move_base_node``` node, which brings up the navigation stack on the robot, must be running. This package (rrt_exploration) generates target exploration goals, each robot must be able to receive these points and move towards them. This is why the navigation stack is needed. Additionally, each robot must have a global and local cost maps. All of these are proivded from the ```move_base_node```. 

### 3.5. A mapping node
Each robot should have a local map generated from the [gmapping](http://wiki.ros.org/gmapping) package.
### 3.6. A map merging node
For the multi-robot case, there should be a node that merges all the local maps into one global map. You can use [this](http://wiki.ros.org/multirobot_map_merge) package.
## 4. Nodes
There are 3 types of nodes; nodes for detecting frontier points in an occupancy grid map, a node for filtering the detected points, and a node for assigning the points to the robots. The following figure shows the structure:
![alt text](https://github.com/hasauino/storage/blob/master/pictures/fullSchematic.png "overview of the exploration strategy")

### 4.1. global_rrt_frontier_detector
The ```global_rrt_frontier_detector``` node takes an occupancy grid and finds frontier points (which are exploration targets) in it. It publishes the detected points so the filter node can process. In multi-robot configuration, it is intended to have only a single instance of this node running. 

Running additional instances of the global frontier detector can enhance the speed of frontier points detection, if needed.
#### 4.1.1. Parameters
 - ```~map_topic``` (string, default: "/robot_1/map"): This parameter defines the topic name on which the node will recieve the map.
  - ```~eta``` (float, default: 0.5): This parameter controls the growth rate of the RRT that is used in the detection of frontier points, the unit is in meters. This parameter should be set according to the map size, a very large value will cause the tree to grow faster and  hence detect frontier points faster, but a large growth rate also implies that the tree will be missing small corners in the map.

#### 4.1.2. Subscribed Topics
 - The map (Topic name is defined by the ```~map_topic``` parameter) ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))

- ```clicked_point``` ([geometry_msgs/PointStamped Message](http://docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html)): The ```global_rrt_frontier_detector``` node requires that the region to be explored is defined. This topic is where the node recieves five points that define the region. The first four points are four defining a square region to be explored, and the last point is the tree starting point. After publishing those five points on this topic, the RRT will start detecting frontier points. The five points are intended to be published from Rviz using ![alt text](https://github.com/hasauino/storage/blob/master/pictures/publishPointRviz_button.png "Rviz publish point button") button.

#### 4.1.3. Published Topics
 - ```detected_points``` ([geometry_msgs/PointStamped Message](http://docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html)): The topic on which the node publishes detected frontier points.

- ```~shapes``` ([visualization_msgs/Marker Message](http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html)): On this topic, the node publishes line shapes to visualize the RRT using Rviz.


### 4.2. local_rrt_frontier_detector
This node is similar to the global_rrt_frontier_detector. However, it works differently, as the tree here keeps resetting every time a frontier point is detected. This node is intended to be run along side the global_rrt_frontier_detector node, it is responsible for fast detection of frontier points that lie in the close vicinity of the robot.

In multi-robot configuration, each robot runs an instance of the local_rrt_frontier_detector. So for a team of 3 robots, there will be 4 nodes for detecting frontier points; 3 local detectors and 1 global detector.
Running additional instances of the local frontier detector can enhance the speed of frontier points detection, if needed.


All detectors will be publishing detected frontier points on the same topic (```/detected_points```).
#### 4.2.1. Parameters
- ```~robot_frame``` (string, default: "/robot_1/base_link"): The frame attached to the robot. Every time the tree resets, it will start from the current robot location obtained from this frame.

 - ```~map_topic``` (string, default: "/robot_1/map"): This parameter defines the topic name on which the node will recieve the map.
  - ```~eta``` (float, default: 0.5): This parameter controls the growth rate of the local RRT.

#### 4.2.2. Subscribed Topics
 - The map (Topic name is defined by the ```~map_topic``` parameter) ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html)).

- ```clicked_point``` ([geometry_msgs/PointStamped Message](http://docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html)): The ```lobal_rrt_frontier_detector``` also subscribes to this topic similar to the global_rrt_frontier_detector. 
#### 4.2.3. Published Topics
 - ```detected_points``` ([geometry_msgs/PointStamped Message](http://docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html)): The topic on which the node publishes detected frontier points.

- ```~shapes``` ([visualization_msgs/Marker Message](http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html)): On this topic, the node publishes line shapes to visualize the RRT using Rviz.


### 4.3. frontier_opencv_detector
This node is another frontier detector, but it is not based on RRT. This node uses OpenCV tools to detect frontier points. It is intended to be run alone, and in multi-robot configuration only one instance should be run (running additional instances of this node does not make any difference).

Originally this node was implemented for comparison against the RRT-based frontier detectors. Running this node along side the RRT detectors (local and global) may enhance the speed of frotiner points detection.


Note: You can run any type and any number of detectors, all the detectors will be publishing on the same topic which the filter node (will be explained in the following section) is subscribing to. on the other hand, the filter will pass the filtered forntier points to the assigner in order to command the robots to explore these points. 

#### 4.3.1. Parameters
 - ```~map_topic``` (string, default: "/robot_1/map"): This parameter defines the topic name on which the node will recieve the map.

#### 4.3.2. Subscribed Topics
 - The map (Topic name is defined by the ```~map_topic``` parameter) ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))

#### 4.3.3. Published Topics
 - ```detected_points``` ([geometry_msgs/PointStamped Message](http://docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html)): The topic on which the node publishes detected frontier points.

- ```shapes``` ([visualization_msgs/Marker Message](http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html)): On this topic, the node publishes detected points to be visualized using Rviz.

### 4.4. filter
The filter nodes receives the detected frontier points from all the detectors, filters the points, and passes them to the assigner node to command the robots. Filtration includes the delection of old and invalid points, and it also dicards redundant points.

#### 4.4.1. Parameters
 - ```~map_topic``` (string, default: "/robot_1/map"): This parameter defines the topic name on which the node will recieve the map. The map is used to know which points are no longer frontier points (old points).
  - ```~costmap_clearing_threshold``` (float, default: 70.0): Any frontier point that has an occupancy value greater than this threshold will be considered invalid. The occupancy value is obtained from the costmap. 
  - ```~info_radius```(float, default: 1.0): The information radius used in calculating the information gain of frontier points.
  - ```~goals_topic``` (string, default: "/detected_points"): defines the topic on which the node receives detcted frontier points.
  - ```~n_robots```(float, default: 1.0): Number of robots.
  - ```~rate```(float, default: 100): node loop rate (in Hz).

#### 4.4.2. Subscribed Topics
 - The map (Topic name is defined by the ```~map_topic``` parameter) ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html)).

- ```robot_x/move_base_node/global_costmap/costmap``` ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html)): where x (in robot_x) refers to robot's number. 

The filter node subscribes for all the costmap topics of all the robots, the costmap is required therefore. Normally, costmaps should be published by the navigation stack (after bringing up the navigation stack on the robots, each robot will have a costmap).
For example, if  ```n_robots=2```, the node will subscribe to:
```robot_1/move_base_node/global_costmap/costmap``` and ```robot_2/move_base_node/global_costmap/costmap```.
The costmaps are used to delete invalid points.

Note: Namespaces of all the nodes corresponding to a robot should start with ```robot_x```. Again ```x``` is the robot number. 

 - The goals topic (Topic name is defined by the ```~goals_topic``` parameter)([geometry_msgs/PointStamped Message](http://docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html)): The topic on which the filter node receives detected frontier points.
 
#### 4.4.3. Published Topics

 - ```frontiers``` ([visualization_msgs/Marker Message](http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html)): The topic on which the filter node publishes the received frontier points for visualiztion on Rviz.
 
 - ```centroids``` ([visualization_msgs/Marker Message](http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html)): The topic on which the filter node publishes only the filtered frontier points for visualiztion on Rviz.

 - ```filtered_points``` ([PointArray](https://github.com/hasauino/rrt_exploration/blob/master/msg/PointArray.msg)): All the filtered points are sent as an array of points to the assigner node on this topic.

### 4.5. Assigner
This node recieve target exploration goals, which are the filtered frontier points published by the filter node, and commands the robots accordingly. The assigner node commands the robots through the ```move_base_node```. This is why you have bring up the navigation stack on your robots.

#### 4.5.1. Parameters
- ```~map_topic``` (string, default: "/robot_1/map"): This parameter defines the topic name on which the node will recieve the map. In the single robot case, this topic should be set to the map topic of the robot. In the multi-robot case, this topic must be set to global merged map.
 - ```~info_radius```(float, default: 1.0): The information radius used in calculating the information gain of frontier points.
  
 - ```~info_multiplier```(float, default: 3.0): The unit is meter. This parameter is used to give importance to information gain of a frontier point over the cost (expected travel distance to a frontier point).
  
- ```~hysteresis_radius```(float, default: 3.0): The unit is meter. This parameter defines the hysteresis radius.

- ```~hysteresis_gain```(float, default: 2.0): The unit is meter. This parameter defines the hysteresis gain.
 
- ```~frontiers_topic``` (string, default: "/filtered_points"): The topic on which the assigner node receives filtered frontier points.

- ```~n_robots```(float, default: 1.0): Number of robots.

-  ```~delay_after_assignement```(float, default: 0.5): The unit is seconds. It defines the amount of delay after each robot assignment.

- ```~rate```(float, default: 100): node loop rate (in Hz).

#### 4.5.2. Subscribed Topics
 - The map (Topic name is defined by the ```~map_topic``` parameter) ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html)).
 
- Filtered frontier points topic (Topic name is defined by the ```~frontiers_topic``` parameter)  ([PointArray](https://github.com/hasauino/rrt_exploration/blob/master/msg/PointArray.msg)).

#### 4.5.3. Published Topics
The assigner node does not publish anything. It sends the assinged point to the ```move_base_node``` using Actionlib (the assigner node is an actionlib client to the move_base_node actionlib server).







