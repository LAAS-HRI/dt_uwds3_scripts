#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy

class FactComputation():

    def __init__(self):
        # ontologiesManipulator =OntologiesManipulator()
        # self.onto = ontologiesManipulator.get("common_ground")


        self.onto = OntologyManipulator("")
        self.global_frame_id = rospy.get_param("~global_frame_id", "map")
        return 0


    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == "__main__":
    rospy.init_node("fact_computation", anonymous=False)
    fact = FactComputation().run()
