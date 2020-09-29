#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import cv2
from tf.transformations import quaternion_from_euler
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from pyuwds3.types.camera import Camera
from pyuwds3.types.vector.vector6d import Vector6D
from pyuwds3.types.vector.vector6d_stable import Vector6DStable
from pyuwds3.types.scene_node import SceneNode
from pyuwds3.types.shape.mesh import Mesh
from uwds3_msgs.msg import WorldStamped
from pyuwds3.utils.tf_bridge import TfBridge
from pyuwds3.utils.view_publisher import ViewPublisher
from pyuwds3.utils.marker_publisher import MarkerPublisher
from pyuwds3.utils.world_publisher import WorldPublisher
from ar_track_alvar_msgs.msg import AlvarMarkers
import yaml
from geometry_msgs.msg import PoseStamped
from ontologenius import OntologiesManipulator
from ontologenius import OntologyManipulator
from pr2_motion_tasks_msgs.srv import GetPose
from pr2_motion_tasks_msgs.srv import GetPoseResponse
from uwds3_msgs.msg import WorldStamped

DEFAULT_SENSOR_QUEUE_SIZE = 1


class TagServiceNode(object):
    def __init__(self):
        """
        """

        self.tag_service = rospy.Service("~getPose",GetPose, self.send_ar_tag, buff_size=65536)
        self.subscriber = rospy.Subscriber("ar_tracks", WorldStamped, self.world_callback)
        self.scene_nodes = {}
        self.header = rospy.Header()

    def world_callback(self, world_msg):
        self.header = world_msg.header
        self.scene_nodes = {}
        for node in world_msg.world.scene:
            self.scene_nodes[node.id] = SceneNode().from_msg(node)




    def send_ar_tag(self, msg):
        ret_list = []
        for id in msg.ids:
            pose_s =PoseStamped()
            if id not in self.scene_nodes.keys():
                pose_s.header.frame_id=''
            else:
                node = self.scene_nodes[id]
                if node.pose is None:
                    pose_s.header.frame_id=''
                else:
                    position = node.pose.pos.position().to_array()
                    pose_s.pose.position.x = position[0]
                    pose_s.pose.position.y = position[1]
                    pose_s.pose.position.z = position[2]

                    orientation = node.pose.quaternion()

                    pose_s.pose.orientation.x = orientation[0]
                    pose_s.pose.orientation.y = orientation[1]
                    pose_s.pose.orientation.z = orientation[2]
                    pose_s.pose.orientation.w = orientation[3]
                    pose_s.header.stamp = node.pose.pos.last_update
                    pose_s.header.frame_id = self.header.frame_id
            ret_list.append(pose_s)
            ret = GetPoseResponse()
            ret.poses = ret_list
        return ret

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == "__main__":
    rospy.init_node("tag_service", anonymous=False)
    perception = TagServiceNode().run()
