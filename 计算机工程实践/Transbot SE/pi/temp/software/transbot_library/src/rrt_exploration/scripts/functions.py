import rospy
import tf
from numpy import array
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from numpy import floor
from numpy.linalg import norm
from numpy import inf


# ________________________________________________________________________________


class robot:
    goal = MoveBaseGoal()
    start = PoseStamped()
    end = PoseStamped()

    def __init__(self,name):
        self.assigned_point = []
        self.global_frame = rospy.get_param('~global_frame', 'map')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self.plan_service = rospy.get_param( '~plan_service', '/move_base/NavfnROS/make_plan')
        self.listener = tf.TransformListener()
        self.listener.waitForTransform(
            self.global_frame, self.robot_frame, rospy.Time(0), rospy.Duration(10.0))
        cond = 0
        while cond == 0:
            try:
                rospy.loginfo('Waiting for the robot transform')
                (trans, rot) = self.listener.lookupTransform(
                    self.global_frame, self.robot_frame, rospy.Time(0))
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                cond = 0
        self.position = array([trans[0], trans[1]])
        self.assigned_point = self.position
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.client.wait_for_server()
        robot.goal.target_pose.header.frame_id = self.global_frame
        robot.goal.target_pose.header.stamp = rospy.Time.now()
        # print "plan_service: ", self.plan_service
        rospy.wait_for_service(self.plan_service)
        self.make_plan = rospy.ServiceProxy( self.plan_service, GetPlan)
        robot.start.header.frame_id = self.global_frame
        robot.end.header.frame_id = self.global_frame

    def getPosition(self):
        cond = 0
        while cond == 0:
            try:
                (trans, rot) = self.listener.lookupTransform(
                    self.global_frame, '/' + self.robot_frame, rospy.Time(0))
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                cond = 0
        self.position = array([trans[0], trans[1]])
        # print ("self.position: ",self.position)
        return self.position

    def sendGoal(self, point):
        robot.goal.target_pose.pose.position.x = point[0]
        robot.goal.target_pose.pose.position.y = point[1]
        robot.goal.target_pose.pose.orientation.w = 1.0
        # print("robot.goal: ",robot.goal)
        self.client.send_goal(robot.goal)
        self.assigned_point = array(point)

    def cancelGoal(self):
        self.client.cancel_goal()
        self.assigned_point = self.getPosition()

    def getState(self):
        return self.client.get_state()

    def makePlan(self, start, end):
        robot.start.pose.position.x = start[0]
        robot.start.pose.position.y = start[1]
        robot.end.pose.position.x = end[0]
        robot.end.pose.position.y = end[1]
        # print("robot.start: ",robot.start)
        # print("robot.end: ",robot.end)
        start = self.listener.transformPose('/map', robot.start)
        end = self.listener.transformPose('/map', robot.end)
        plan = self.make_plan(start=start, goal=end, tolerance=0.0)
        return plan.plan.poses


# ________________________________________________________________________________


def index_of_point(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y
    width = mapData.info.width
    Data = mapData.data
    index = int((floor((Xp[1] - Xstarty) / resolution) *
                 width) + (floor((Xp[0] - Xstartx) / resolution)))
    return index


def point_of_index(mapData, i):
    y = mapData.info.origin.position.y + \
        (i / mapData.info.width) * mapData.info.resolution
    x = mapData.info.origin.position.x + \
        (i - (i / mapData.info.width) * (mapData.info.width)) * mapData.info.resolution
    return array([x, y])


# ________________________________________________________________________________


def informationGain(mapData, point, r):
    infoGain = 0
    index = index_of_point(mapData, point)
    r_region = int(r / mapData.info.resolution)
    init_index = index - r_region * (mapData.info.width + 1)
    for n in range(0, 2 * r_region + 1):
        start = n * mapData.info.width + init_index
        end = start + 2 * r_region
        limit = ((start / mapData.info.width) + 2) * mapData.info.width
        for i in range(start, end + 1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                if (mapData.data[i] == -1 and norm(array(point) - point_of_index(mapData, i)) <= r):
                    infoGain += 1
    return infoGain * (mapData.info.resolution ** 2)


def checkAround(mapData, point, r):
    valueAtound = 0
    index = index_of_point(mapData, point)
    r_region = int(r / mapData.info.resolution)
    init_index = index - r_region * (mapData.info.width + 1)
    for n in range(0, 2 * r_region + 1):
        start = n * mapData.info.width + init_index
        end = start + 2 * r_region
        # limit = ((start/mapData.info.width)+2)*mapData.info.width
        for i in range(start, end + 1):
            if (i >= 0 and i < len(mapData.data)):
                if (mapData.data[i] > 0):
                    valueAtound += mapData.data[i]
    return valueAtound


# ________________________________________________________________________________


def discount(mapData, assigned_pt, centroids, infoGain, r):
    index = index_of_point(mapData, assigned_pt)
    r_region = int(r / mapData.info.resolution)
    init_index = index - r_region * (mapData.info.width + 1)
    for n in range(0, 2 * r_region + 1):
        start = n * mapData.info.width + init_index
        end = start + 2 * r_region
        limit = ((start / mapData.info.width) + 2) * mapData.info.width
        for i in range(start, end + 1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                for j in range(0, len(centroids)):
                    current_pt = centroids[j]
                    if (mapData.data[i] == -1 and
                        norm(point_of_index(mapData, i) - current_pt) <= r and
                        norm(point_of_index(mapData, i) - assigned_pt) <= r):
                        # this should be modified, subtract the area of a cell, not 1
                        infoGain[j] -= 1
    return infoGain


# ________________________________________________________________________________


def pathCost(path):
    if (len(path) > 0):
        i = len(path) / 2
        p1 = array([path[i - 1].pose.position.x, path[i - 1].pose.position.y])
        p2 = array([path[i].pose.position.x, path[i].pose.position.y])
        return norm(p1 - p2) * (len(path) - 1)
    else:
        return inf


# ________________________________________________________________________________


def unvalid(mapData, pt):
    index = index_of_point(mapData, pt)
    r_region = 5
    init_index = index - r_region * (mapData.info.width + 1)
    for n in range(0, 2 * r_region + 1):
        start = n * mapData.info.width + init_index
        end = start + 2 * r_region
        limit = ((start / mapData.info.width) + 2) * mapData.info.width
        for i in range(start, end + 1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                if (mapData.data[i] == 1):
                    return True
    return False


# ________________________________________________________________________________


def Nearest(V, x):
    n = inf
    i = 0
    for i in range(0, V.shape[0]):
        n1 = norm(V[i, :] - x)
        if (n1 < n):
            n = n1
            result = i
    return result


# ________________________________________________________________________________


def Nearest2(V, x):
    n = inf
    result = 0
    for i in range(0, len(V)):
        n1 = norm(V[i] - x)

        if (n1 < n):
            n = n1
    return i


# ________________________________________________________________________________


def gridValue(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y

    width = mapData.info.width
    Data = mapData.data
    # returns grid value at "Xp" location
    # map data:  100 occupied      -1 unknown       0 free
    index = (floor((Xp[1] - Xstarty) / resolution) * width) + \
            (floor((Xp[0] - Xstartx) / resolution))

    if int(index) < len(Data):
        return Data[int(index)]
    else:
        return 100
