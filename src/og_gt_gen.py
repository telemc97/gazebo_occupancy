import os
import sys
import rospkg
import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates

sys.path.append(os.path.join(os.path.dirname(__file__), "tools"))

from tools.occupancy import populate_occupancy


pkg = rospkg.RosPack()
package_path = pkg.get_path('gazebo_occupancy')

class og_gt_gen:
    def __init__(self):

        self.model_state_topic = rospy.get_param('~model_state_topic', '/gazebo/model_states')

        self.map_size = rospy.get_param('~map_size', 100)
        self.resolution = rospy.get_param('~resolution', 1.0)

        self.map = np.zeros(shape=(self.map_size,self.map_size), dtype=int)
        self.mapOriginOffset = np.zeros(shape=(3,1), dtype=int)

        rospy.Subscriber(self.model_state_topic, ModelStates, self.mainCallback, queue_size=2)


    def mainCallback(self, models):
        names = np.asarray(models.name, dtype=object)
        pos = np.asarray(models.pose, dtype=object)
        entries = np.stack((names,pos), 1)

        np.apply_along_axis(self.originOffsetCalc, 1, entries)
        np.apply_along_axis(self.coordGen, 1, entries)
        
        occupancyGrid = populate_occupancy(self.map, self.resolution, self.mapOriginOffset, rospy.Time.now())
        occupancyGrid.populate()

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
            else:
                print(grid_x,grid_y)


if __name__ == "__main__":    
    rospy.init_node("gazebo_occupancy", anonymous=True)

    r = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        og_gt_gen()
        r.sleep()