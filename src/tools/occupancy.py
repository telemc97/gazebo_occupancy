import os
import sys
import numpy as np
import rospkg
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from geometry_msgs.msg import Pose

class populate_occupancy:
    def __init__(self, array, res, stamp, origin = [0,0,0]):
        self.array = np.asarray(array, dtype=int)
        self.res = res
        self.stamp = stamp

        self.origin = Pose()
        self.origin.position.x = origin[0]
        self.origin.position.y = origin[1]
        self.origin.position.z = origin[2]

        self.OccupancyGrid_topic = rospy.get_param('~OccupancyGrid_topic', 'Ground_Truth_Occupancy_Grid')
        self.OccupancyGrid_frame = rospy.get_param('~OccupancyGrid_frame', 'map')
        self.publisher_Occupancy_Grid = rospy.Publisher(self.OccupancyGrid_topic, OccupancyGrid, queue_size=2)

    def populate(self):
        header = Header()
        header.frame_id = self.OccupancyGrid_frame
        header.stamp = self.stamp

        mapMetaData = MapMetaData()
        mapMetaData.resolution = self.res
        mapMetaData.width = self.array.shape[1]
        mapMetaData.height = self.array.shape[0]
        mapMetaData.origin = self.origin
        
        occupancyGrid = OccupancyGrid()

        for i in np.ndindex(self.array.shape):
            occupancyGrid.data.append(self.array[i])
        
        occupancyGrid.header = header
        occupancyGrid.info = mapMetaData

        self.publisher_Occupancy_Grid.publish(occupancyGrid)