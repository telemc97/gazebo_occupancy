#!/usr/bin/env python3
import rospy
import rospkg
import numpy as np
from gazebo_occupancy.msg import GroundTruthDebug
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from tools.occupancy import populate_occupancy

pkg = rospkg.RosPack()
package_path = pkg.get_path('gazebo_occupancy')

class iouCalculator:
    def __init__ (self, gt_mapArray, occupancy_grid_msg_, map_origin_offset_, map_resolution_, debugData_):
        
        self.gt_mapArray = gt_mapArray
        self.occupancyGrid = occupancy_grid_msg_
        self.gt_mapArray_origin_offset = map_origin_offset_
        self.gt_mapArray_res = map_resolution_
        self.debugData = debugData_

        self.OccupancyGrid_cropped_topic = '~Ground_Truth_Occupancy_Grid_Cropped'
        self.OccupancyGrid_cropped_frame = rospy.get_param('~Ground_Truth_Occupancy_Grid_Cropped_frame', 'map')

        self.publisher_iou = rospy.Publisher('Ground_Truth_Debug', GroundTruthDebug, queue_size=2)

        self.rate = rospy.get_param('~exec_rate', 0.5)
        self.debug = rospy.get_param('~debug', True)
        self.r = rospy.Rate(self.rate)


    def calculateIoU(self):

        starting_idx = self.getArrayStartIndex(self.gt_mapArray_origin_offset, self.occupancyGrid.info)
        ending_idx = (starting_idx[0]+self.occupancyGrid.info.width), (starting_idx[1]+self.occupancyGrid.info.height)

        gt_array = self.gt_mapArray[starting_idx[0]:ending_idx[0], starting_idx[1]:ending_idx[1]]
        grid_array = self.toMatrix(self.occupancyGrid)

        gt_array_bool = self.boolConverter(gt_array)
        grid_array_bool = self.boolConverter(grid_array)
        iou = self.iouCalc(gt_array_bool, grid_array_bool)

        if self.debug:
            if not rospy.is_shutdown():
                occupancyGridCreator = populate_occupancy(gt_array, self.gt_mapArray_res, self.occupancyGrid.info.origin, rospy.Time.now(), self.OccupancyGrid_cropped_topic, self.OccupancyGrid_cropped_frame)
                occupancyGridCreator.populate()
                self.populateMsg(iou)
                self.r.sleep()
    

    def populateMsg(self, iou_):
        header = Header()
        groundTruthDebug = GroundTruthDebug()
        header.stamp = rospy.Time.now()
        groundTruthDebug.header = header
        groundTruthDebug.valid_percantage = self.debugData[0]
        groundTruthDebug.land_points_sum = self.debugData[1]
        groundTruthDebug.iou = iou_
        groundTruthDebug
        self.publisher_iou.publish(groundTruthDebug)


    def iouCalc(self, array0, array1):
        overlap = array0*array1
        union = array0+array1
        iou_ = overlap.sum()/float(union.sum())
        return iou_


    def boolConverter(self, array):
        bool_array = np.zeros(shape=(array.shape), dtype=bool)
        it = np.nditer(array, flags=['multi_index'])
        for x in it:
            if (x==100):
                bool_array[it.multi_index]=1
        return bool_array


    def getIndexFromLinearIndex(self, rows_, index):
        y = int(index/rows_)
        x = int(index-(y*rows_))
        return x, y


    def toMatrix(self, occupancyGrid_):
        cols = occupancyGrid_.info.height
        rows = occupancyGrid_.info.width
        output_matrix = np.zeros(shape=(rows,cols),dtype=int)
        input_matrix = np.asarray(occupancyGrid_.data)
        for i in range(input_matrix.shape[0]):
            if (input_matrix[i]==100):
                idx_x, idx_y = self.getIndexFromLinearIndex(rows, i)
                output_matrix[idx_x, idx_y]=100
        return output_matrix


    def getArrayStartIndex(self, gt_mapArray_origin_offset_, occupancyGridInfo):
        start_idx_x = int(abs(gt_mapArray_origin_offset_.position.x-occupancyGridInfo.origin.position.x))
        start_idx_y = int(abs(gt_mapArray_origin_offset_.position.y-occupancyGridInfo.origin.position.y))
        starting_idx_ = start_idx_x, start_idx_y
        return starting_idx_