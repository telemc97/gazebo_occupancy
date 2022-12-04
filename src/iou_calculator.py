import os
import sys
import rospkg
import rospy
import message_filters
import numpy as np

from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData

sys.path.append(os.path.join(os.path.dirname(__file__), "tools"))

from tools.occupancy import populate_occupancy


pkg = rospkg.RosPack()
package_path = pkg.get_path('gazebo_occupancy')

class iou_calculator:
    def __init__(self):

        self.groundTruth_topic = rospy.get_param('~Ground_Truth_Occupancy_Grid', 'Ground_Truth_Occupancy_Grid')
        self.occupancyGrid_topic = rospy.get_param('~Occupancy_Grid', 'occupancy_output')

        self.OccupancyGrid_cropped_topic = rospy.get_param('~Ground_Truth_Occupancy_Grid_Cropped', 'Ground_Truth_Occupancy_Grid_Cropped')
        self.OccupancyGrid_cropped_frame = rospy.get_param('~Ground_Truth_Occupancy_Grid_Cropped_frame', 'map')

        self.rate = rospy.get_param('~exec_rate', 0.5)
        self.r = rospy.Rate(self.rate)

        self.groundTruth = message_filters.Subscriber(self.groundTruth_topic, OccupancyGrid)
        self.occupancyGrid = message_filters.Subscriber(self.occupancyGrid_topic, OccupancyGrid)

        self.synch = message_filters.ApproximateTimeSynchronizer([self.groundTruth, self.occupancyGrid], queue_size=10, slop=0.5)
        self.synch.registerCallback(self.mainCallback)

    def mainCallback(self, groundTruth_, occupancyGrid_):
        gt_array = self.toMatrix(groundTruth_)
        gt_bounds = self.getMatrixBounds(groundTruth_)
        grid_array = self.toMatrix(occupancyGrid_)
        grid_array_bounds = self.getMatrixBounds(occupancyGrid_)
        starting_idx = (gt_bounds[0]-grid_array_bounds[0]), (gt_bounds[2]-grid_array_bounds[2])
        ending_idx = (starting_idx[0]+occupancyGrid_.info.width), (starting_idx[1]+occupancyGrid_.info.height)
        gt_array = np.transpose(gt_array[starting_idx[0]:ending_idx[0], starting_idx[1]:ending_idx[1]])

        if not rospy.is_shutdown():
            occupancyGrid = populate_occupancy(gt_array, groundTruth_.info.resolution, occupancyGrid_.info.origin, rospy.Time.now(), self.OccupancyGrid_cropped_topic, self.OccupancyGrid_cropped_frame)
            occupancyGrid.populate()
            self.r.sleep()



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

    def getMatrixBounds(self, occupancyGrid_):
        width_left = int(abs(occupancyGrid_.info.origin.position.x))
        width_right = int(occupancyGrid_.info.width + occupancyGrid_.info.origin.position.x)
        cols_down = int(abs(occupancyGrid_.info.origin.position.y))
        cols_up = int(occupancyGrid_.info.height + occupancyGrid_.info.origin.position.y)
        bounds = width_left, width_right, cols_down, cols_up
        return bounds

if __name__ == "__main__":    
    rospy.init_node("iou_calculator", anonymous=True)
    iou_calculator()
    rospy.spin()
