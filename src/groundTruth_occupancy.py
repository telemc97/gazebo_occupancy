#!/usr/bin/env python3
import rospy
import numpy as np
import math
import rospkg
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseStamped
import message_filters
from gazebo_occupancy.msg import LandingTarget
from gazebo_occupancy.msg import LandingTargets
from tools.occupancy import populate_occupancy
from iou_calculator import iouCalculator
from nav_msgs.msg import OccupancyGrid
from mavros_msgs.msg import ExtendedState


pkg = rospkg.RosPack()
package_path = pkg.get_path('gazebo_occupancy')

class groundTruthOccupancy:
    def __init__(self):
        self.map_size = rospy.get_param('~map_size', 300)
        self.resolution = rospy.get_param('~resolution', 1.0)
        self.rate = rospy.get_param('~exec_rate', 0.5)
        self.safety_radius = rospy.get_param('~safety_radius', 2.0)
        self.r = rospy.Rate(self.rate)

        #Subscriber Topic Names
        self.model_state_topic = rospy.get_param('~model_state_topic', '/gazebo/model_states')
        self.OccupancyGrid_topic = rospy.get_param('~Occupancy_Grid', '/occupancy_output')
        self.land_points_topic = rospy.get_param('~land_points_topic', '/Debug_Landing_Targets')
        #Mavros Subscriber Names
        self.extended_state_topic =  '/mavros/extended_state'
        self.local_position_pose_topic = '/mavros/local_position/pose'

        #Frame Names
        self.Ground_Truth_OccupancyGrid_frame = rospy.get_param('~Ground_Truth_Occupancy_Grid_frame', 'map')
        self.OccupancyGrid_frame = rospy.get_param('~Ground_Truth_Occupancy_Grid_frame', 'map')

        #Publisher Names
        self.Ground_Truth_OccupancyGrid_topic = rospy.get_param('~Ground_Truth_Occupancy_Grid', 'Ground_Truth_Occupancy_Grid')

        self.offset_x = -(self.map_size/2)
        self.offset_y = -(self.map_size/2)
        self.offset_z = 0

        self.map = np.zeros(shape=(self.map_size,self.map_size), dtype=int)
        self.map1 = np.zeros(shape=(self.map_size,self.map_size), dtype=int)

        self.mapOriginOffset = np.array([self.offset_x, self.offset_y, self.offset_z], dtype=int)

        self.models_state = message_filters.Subscriber(self.model_state_topic, ModelStates)
        self.land_points = message_filters.Subscriber(self.land_points_topic, LandingTargets)  
        self.occupancyGrid = message_filters.Subscriber(self.OccupancyGrid_topic, OccupancyGrid)

        self.extended_state = message_filters.Subscriber(self.extended_state_topic, ExtendedState)
        self.local_position_pose = message_filters.Subscriber(self.local_position_pose_topic, PoseStamped)

        self.synch = message_filters.ApproximateTimeSynchronizer([self.models_state, self.land_points, self.occupancyGrid, self.extended_state, self.local_position_pose], queue_size=10, slop=1, allow_headerless=True)
        self.synch.registerCallback(self.mainCallback)


    def getMap(self):
        mapArray_ = self.map.T
        return mapArray_

    def getLandPoints(self):
        landPoints_ = self.land_points
        return landPoints_

    def getDebugData(self):
        land_points_sum_, percentage_ = self.land_poinst_sum, self.percentage
        return percentage_, land_points_sum_


    def mainCallback(self, models_msg, land_points_msg, occupancy_grid_msg, extended_state_msg, local_position_msg):

        self.land_points = np.zeros(shape=(0,0), dtype=object)
        self.land_points = np.asarray(land_points_msg.landing_targets, dtype=object)

        names = np.asarray(models_msg.name, dtype=object)
        pos = np.asarray(models_msg.pose, dtype=object)
        entries = np.stack((names,pos), 1)

        np.apply_along_axis(self.originOffsetCalc, 1, entries)
        np.apply_along_axis(self.coordGen, 1, entries)

        mapOriginOffsetPose = Pose()
        mapOriginOffsetPose.position.x = self.mapOriginOffset[0]
        mapOriginOffsetPose.position.y = self.mapOriginOffset[1]
        mapOriginOffsetPose.position.z = self.mapOriginOffset[2]

        if (extended_state_msg.landed_state == 1):
            robot_position = local_position_msg.pose.position.x, local_position_msg.pose.position.y
            min_dist = self.getMinDistance(robot_position)
        else:
            min_dist = 2310
        

        mapArray = self.getMap()
        land_points = self.getLandPoints()
        self.land_poinst_sum, self.percentage = self.checkOccupiedPercentage(land_points, mapArray, occupancy_grid_msg.info.origin)
        debugData = self.getDebugData()

        if not rospy.is_shutdown():
            occupancyGridCreator = populate_occupancy(mapArray, self.resolution, mapOriginOffsetPose, rospy.Time.now(), self.Ground_Truth_OccupancyGrid_topic, self.Ground_Truth_OccupancyGrid_frame)
            occupancyGridCreator.populate()

            iou_calculator = iouCalculator(mapArray, occupancy_grid_msg, mapOriginOffsetPose, self.resolution, debugData, min_dist)
            iou_calculator.calculateIoU()

            self.r.sleep()


    def originOffsetCalc(self, array):
        if ((array[0])[:5] == "Nurse"):

            grid_x = int(array[1].position.y / self.resolution)
            if (grid_x<self.mapOriginOffset[0]):
                self.mapOriginOffset[0] = grid_x

            grid_y = int(array[1].position.x / self.resolution)
            if (grid_y<self.mapOriginOffset[1]):
                self.mapOriginOffset[1] = grid_y


    def coordGen(self, array):
        if ((array[0])[:5] == "Nurse"):
            grid_x = int((array[1].position.y+abs(self.mapOriginOffset[0]) / self.resolution))
            grid_y = int((array[1].position.x+abs(self.mapOriginOffset[1]) / self.resolution))
            if ( (grid_x>=0 and grid_x<=self.map.shape[0]) and (grid_y>=0 and grid_y<=self.map.shape[1]) ):
                self.map[grid_x, grid_y] = 100
                self.map1[grid_x, grid_y] = 100
            idx = (grid_x, grid_y)
            self.applySafetyArea(idx)


    def applySafetyArea(self, idx_):
        i = int(idx_[0] - self.safety_radius)
        j = int(idx_[1] - self.safety_radius)
        i_max = int(idx_[0] + self.safety_radius)+2
        j_max = int(idx_[1] + self.safety_radius)+2
        self.map[i:i_max, j:j_max] = 100


    def checkOccupiedPercentage(self, land_points, mapArray_, origin_):
        valid_land_points = 0
        for i in range(len(land_points)):
            x = land_points[i].x - int((self.offset_x) - origin_.position.x) #ToDo: make them depended on the resolution
            y = land_points[i].y - int((self.offset_y) - origin_.position.y)
            if (mapArray_[x,y]==100):
                mapArray_[x,y]=-128
            elif (mapArray_[x,y]==0):
                mapArray_[x,y]=-7
                valid_land_points+=1
        valid_percentage = (100*valid_land_points)/len(land_points)
        return len(land_points), valid_percentage


    def getMinDistance(self, robot_position_):
        it = np.nditer(self.map1, flags=['multi_index'], order='F')
        min_dist_ = 123456789
        for x in it:
            if (x==100):
                dist = self.euclidianDistance(robot_position_, it.multi_index)
                if (dist<min_dist_):
                    min_dist_ = dist
        return min_dist_

    
    def euclidianDistance(self, pt0, pt1):
        dist_ = math.sqrt(pow((pt1[0]-pt0[0]), 2)+pow((pt1[1]-pt0[1]),2))
        dist_ = round(dist_)
        return dist_
        