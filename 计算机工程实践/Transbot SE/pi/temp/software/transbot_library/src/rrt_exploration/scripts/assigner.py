#!/usr/bin/env python

# --------Include modules---------------
import rospy
from copy import copy
from numpy import array
from numpy.linalg import norm
from transbot_msgs.msg import PointArray
from nav_msgs.msg import OccupancyGrid, Odometry
from functions import robot, informationGain, discount, index_of_point
# Subscribers' callbacks------------------------------
mapData = OccupancyGrid()
frontiers = []
global1 = OccupancyGrid()
global2 = OccupancyGrid()
global3 = OccupancyGrid()
globalmaps = []
count = 0


def callBack(data):
    global frontiers
    frontiers = []
    for point in data.points:
        frontiers.append(array([point.x, point.y]))


def mapCallBack(data):
    global mapData
    mapData = data


# def odomCallBack(data):
#     global count
#     if(len(frontiers) != 0):
# 	    if(data.twist.twist.linear.x < 0.005 and data.twist.twist.angular.z < 0.005):
# 	    	count += 1
# 	    else:
# 	    	count = 0

# Node----------------------------------------------

def node():
    global frontiers, mapData, global1, global2, global3, globalmaps, count
    rospy.init_node('assigner', anonymous=False)
    # fetching all parameters
    map_topic = rospy.get_param('~map_topic', '/map')
    # this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
    info_radius = rospy.get_param('~info_radius', 1.0)
    info_multiplier = rospy.get_param('~info_multiplier', 3.0)
    # at least as much as the laser scanner range
    hysteresis_radius = rospy.get_param('~hysteresis_radius', 3.0)
    # bigger than 1 (biase robot to continue exploring current region
    hysteresis_gain = rospy.get_param('~hysteresis_gain', 2.0)
    frontiers_topic = rospy.get_param('~frontiers_topic', '/filtered_points')
    n_robots = rospy.get_param('~n_robots', 1)
    namespace = rospy.get_param('~namespace', '')
    namespace_init_count = rospy.get_param('namespace_init_count', 1)
    delay_after_assignement = rospy.get_param('~delay_after_assignement', 0.5)
    rateHz = rospy.get_param('~rate', 100)
    rate = rospy.Rate(rateHz)
    # -------------------------------------------
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
    rospy.Subscriber(frontiers_topic, PointArray, callBack)
    # rospy.Subscriber('/odom', Odometry, odomCallBack)
    # ---------------------------------------------------------------------------------------------------------------
    # wait if no frontier is received yet
    while len(frontiers) < 1: pass
    centroids = copy(frontiers)
    # wait if map is not received yet
    while (len(mapData.data) < 1):
        pass

    robots = []
    if len(namespace) > 0:
        for i in range(0, n_robots):
            robots.append(robot(namespace + str(i + namespace_init_count)))
    elif len(namespace) == 0:
        robots.append(robot(namespace))
    for i in range(0, n_robots):
        robots[i].sendGoal(robots[i].getPosition())
    # print ("robots: ",robots)
    # -------------------------------------------------------------------------
    # ---------------------     Main   Loop     -------------------------------
    # -------------------------------------------------------------------------
    while not rospy.is_shutdown():
        centroids = copy(frontiers)
        # print("centroids: ", centroids)
        # -------------------------------------------------------------------------
        # Get information gain for each frontier point
        infoGain = []
        for ip in range(0, len(centroids)):
            infoGain.append(informationGain(mapData, [centroids[ip][0], centroids[ip][1]], info_radius))
        # -------------------------------------------------------------------------
        # get number of available/busy robots
        na = []  # available robots
        nb = []  # busy robots
        for i in range(0, n_robots):
            if (robots[i].getState() == 1):
                nb.append(i)
            else:
                na.append(i)

        # rospy.loginfo("available robots: "+str(na))
        # -------------------------------------------------------------------------
        # get dicount and update informationGain
        for i in nb + na:
            infoGain = discount(mapData, robots[i].assigned_point, centroids, infoGain, info_radius)
        # -------------------------------------------------------------------------
        revenue_record = []
        centroid_record = []
        id_record = []

        for ir in na:
            for ip in range(0, len(centroids)):
                cost = norm(robots[ir].getPosition() - centroids[ip])
                threshold = 1
                information_gain = infoGain[ip]
                if (norm(robots[ir].getPosition() - centroids[ip]) <= hysteresis_radius):
                    information_gain *= hysteresis_gain
                revenue = information_gain * info_multiplier - cost
                revenue_record.append(revenue)
                centroid_record.append(centroids[ip])
                id_record.append(ir)

        if len(na) < 1:
            revenue_record = []
            centroid_record = []
            id_record = []
            for ir in nb:
                for ip in range(0, len(centroids)):
                    cost = norm(robots[ir].getPosition() - centroids[ip])
                    threshold = 1
                    information_gain = infoGain[ip]
                    if (norm(robots[ir].getPosition() - centroids[ip]) <= hysteresis_radius):
                        information_gain *= hysteresis_gain

                    if ((norm(centroids[ip] - robots[ir].assigned_point)) < hysteresis_radius):
                        information_gain = informationGain(mapData, [centroids[ip][0], centroids[ip][1]],
                                                           info_radius) * hysteresis_gain

                    revenue = information_gain * info_multiplier - cost
                    revenue_record.append(revenue)
                    centroid_record.append(centroids[ip])
                    id_record.append(ir)
        if (len(revenue_record) != 0):
            rospy.loginfo("revenue record: " + str(revenue_record))
            rospy.loginfo("centroid record: " + str(centroid_record))
        # rospy.loginfo("robot IDs record: "+str(id_record))

        # -------------------------------------------------------------------------
        if (len(id_record) > 0):
            winner_id = revenue_record.index(max(revenue_record))
            robots[id_record[winner_id]].sendGoal(centroid_record[winner_id])

            # print("\n")
            print(centroid_record[winner_id])
            # print(count)
            # print("\n")
            # rospy.loginfo("centroid record: "+str(centroid_record))
            # if (count > 300):
            # 	centroid_record = delete(centroid_record, (winner_id), axis=0)
            # 	revenue_record = delete(revenue_record, (winner_id))
            # 	rospy.loginfo("centroid record: "+str(centroid_record))
            # 	rospy.loginfo("revenue record: "+str(revenue_record))
            # 	count = 0
            # 	winner_id=revenue_record.index(max(revenue_record))
            # robots[id_record[winner_id]].sendGoal(centroid_record[winner_id])

            rospy.loginfo(namespace + str(namespace_init_count + id_record[winner_id]) + "  assigned to  " + str(
                centroid_record[winner_id]))
            rospy.sleep(delay_after_assignement)
        # -------------------------------------------------------------------------
        rate.sleep()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
