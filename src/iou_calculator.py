import rospy
import message_filters
import numpy as np
from gazebo_occupancy.msg import IoU

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

from tools.occupancy import populate_occupancy

class iouCalculator:
    def __init__(self, mapArray):

        self.groundTruth_topic = rospy.get_param('~Ground_Truth_Occupancy_Grid', 'Ground_Truth_Occupancy_Grid')
        self.occupancyGrid_topic = rospy.get_param('~Occupancy_Grid', 'occupancy_output')

        self.OccupancyGrid_cropped_topic = '~Ground_Truth_Occupancy_Grid_Cropped'
        self.OccupancyGrid_cropped_frame = rospy.get_param('~Ground_Truth_Occupancy_Grid_Cropped_frame', 'map')

        self.iou_topic = 'IoU'

        self.publisher_iou = rospy.Publisher(self.iou_topic, IoU, queue_size=2)

        self.rate = rospy.get_param('~exec_rate', 0.5)
        self.debug = rospy.get_param('~debug', True)
        self.r = rospy.Rate(self.rate)

        self.groundTruth = message_filters.Subscriber(self.groundTruth_topic, OccupancyGrid)
        self.occupancyGrid = message_filters.Subscriber(self.occupancyGrid_topic, OccupancyGrid)
        self.mapArray = mapArray

        self.synch = message_filters.ApproximateTimeSynchronizer([self.groundTruth, self.occupancyGrid], queue_size=10, slop=0.5)
        self.synch.registerCallback(self.mainCallback)


    def mainCallback(self, groundTruth_, occupancyGrid_):
        gt_array = self.mapArray
        starting_idx = self.getArrayStartIndex(groundTruth_.info, occupancyGrid_.info)
        ending_idx = (starting_idx[0]+occupancyGrid_.info.width), (starting_idx[1]+occupancyGrid_.info.height)

        gt_array = gt_array[starting_idx[0]:ending_idx[0], starting_idx[1]:ending_idx[1]]
        grid_array = self.toMatrix(occupancyGrid_)

        gt_array_bool = self.boolConverter(gt_array)
        grid_array_bool = self.boolConverter(grid_array)
        iou = self.iouCalc(gt_array_bool, grid_array_bool)
        self.populateMsg(iou)

        if self.debug:
            if not rospy.is_shutdown():
                occupancyGrid = populate_occupancy(gt_array, groundTruth_.info.resolution, occupancyGrid_.info.origin, rospy.Time.now(), self.OccupancyGrid_cropped_topic, self.OccupancyGrid_cropped_frame)
                occupancyGrid.populate()
                self.r.sleep()
    
    def populateMsg(self, iou_):
        header = Header()
        iou = IoU()
        header.stamp = rospy.Time.now()
        iou.iou = iou_
        iou.header = header
        self.publisher_iou.publish(iou)


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

    def getArrayStartIndex(self, groundTruthInfo, occupancyGridInfo):
        start_idx_x = int(abs(groundTruthInfo.origin.position.x-occupancyGridInfo.origin.position.x))
        start_idx_y = int(abs(groundTruthInfo.origin.position.y-occupancyGridInfo.origin.position.y))
        starting_idx_ = start_idx_x, start_idx_y
        return starting_idx_