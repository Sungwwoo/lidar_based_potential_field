#! /usr/bin/env python3

import rospy
from lidar_based_potential_field.basic_pf import BasicAPF
import tf2_ros

if __name__ == "__main__":
    rospy.init_node("basic_APF_test_node")
    apf = BasicAPF()

    rospy.sleep(1)
    apf.run()
    while not rospy.is_shutdown():
        forces = apf.get_forces()
        rospy.loginfo("Forces: %f, %f ,%f", forces[0], forces[1], forces[2])
        weights = apf.get_weights()
        rospy.loginfo("Weights: %d, %d", weights[0], weights[1])
        apf.set_weights(20, 200)
        
