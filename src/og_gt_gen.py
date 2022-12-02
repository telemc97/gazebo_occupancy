import os
import sys
import rospkg
import rospy
import numpy as np
import itertools
from gazebo_msgs.msg import ModelStates

sys.path.append(os.path.join(os.path.dirname(__file__), "tools"))

from tools.occupancy import populate_occupancy


pkg = rospkg.RosPack()
package_path = pkg.get_path('gazebo_occupancy')

class og_gt_gen:
    def __init__(self):

        self.model_state_topic = rospy.get_param('~model_state_topic', '/gazebo/model_states')

        self.map_size = rospy.get_param('~map_size', 200)
        self.resolution = rospy.get_param('~resolution', 1.0)

        rospy.Subscriber(self.model_state_topic, ModelStates, self.mainCallback, queue_size=2)
    


    def mainCallback(self, models):
        models = ModelStates()
        entries = np.zeros(shape=(0,3), dtype=object)
        map = np.zeros(shape=(self.map_size,self.map_size), dtype=int)

        for name, pos in itertools.product(range(len(models.name)), range(len(models.pose))):
            if (name[:5] == "Nurse"):
                entries = np.vstack(entries, np.array([name.name, pos.position.x, pos.position.y, pos.position.z], dtype=object))
                grid_x = int(pos.position.x / self.resolution)
                grid_y = int(pos.position.y / self.resolution)
                map[grid_x, grid_y] = 100
        
        occupancyGrid = populate_occupancy(map, self.resolution, rospy.Time.now())
        occupancyGrid.populate()



if __name__ == "__main__":    
    rospy.init_node("gazebo_occupancy", anonymous=True)

    r = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        og_gt_gen()
        r.sleep()