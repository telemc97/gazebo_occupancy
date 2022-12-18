#!/usr/bin/env python3
import os
import sys
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header

class populate_occupancy:
    def __init__(self, array, res, origin, stamp, topic, frame):
        self.array = array
        self.res = res
        self.stamp = stamp
        self.topic = topic
        self.frame = frame

        self.origin = origin

        self.publisher_Occupancy_Grid = rospy.Publisher(self.topic, OccupancyGrid, queue_size=2)

    def populate(self):
        header = Header()
        header.frame_id = self.frame
        header.stamp = self.stamp

        mapMetaData = MapMetaData()
        mapMetaData.resolution = self.res
        mapMetaData.width = self.array.shape[0]
        mapMetaData.height = self.array.shape[1]
        mapMetaData.origin = self.origin
        
        occupancyGrid = OccupancyGrid()
        
        it = np.nditer(self.array, order='F')
        for x in it:
            occupancyGrid.data.append(x)

        occupancyGrid.header = header
        occupancyGrid.info = mapMetaData

        self.publisher_Occupancy_Grid.publish(occupancyGrid)