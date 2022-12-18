#!/usr/bin/env python3
import rospkg
import rospy

pkg = rospkg.RosPack()
package_path = pkg.get_path('gazebo_occupancy')

from groundTruth_occupancy import groundTruthOccupancy

if __name__ == "__main__":    
    rospy.init_node("ground_truth_occupancy_grid", anonymous=True)
    groundTruth = groundTruthOccupancy()
    rospy.spin()