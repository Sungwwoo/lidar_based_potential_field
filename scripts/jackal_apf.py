#! /usr/bin/env python3


from lidar_based_potential_field.basic_pf import BasicAPF
import rospy

if __name__ == "__main__":
    rospy.init_node("pf_main")

    apf = BasicAPF()

    rospy.sleep(1)
    apf.run()
    while not rospy.is_shutdown():
        rospy.spin()
