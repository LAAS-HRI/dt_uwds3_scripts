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

DEFAULT_SENSOR_QUEUE_SIZE = 1


class ArPerceptionNode(object):
    def __init__(self):
        """
        """

        # self.tag_service = rospy.Service("~getPose",GetPose, self.send_ar_tag, buff_size=65536)

        self.tf_bridge = TfBridge()


        # ontologiesManipulator =OntologiesManipulator()
        # self.onto = ontologiesManipulator.get("common_ground")


        self.onto = OntologyManipulator("")
        self.global_frame_id = rospy.get_param("~global_frame_id", "map")

        # self.bridge = CvBridge()
        # self.robot_camera = None
        # self.camera_info = None

        # self.events = []

        # self.robot_camera_clipnear = rospy.get_param("~robot_camera_clipnear", 0.1)
        # self.robot_camera_clipfar = rospy.get_param("~robot_camera_clipfar", 25.0)

        self.publish_tf = rospy.get_param("~publish_tf", False)

        # self.publish_viz = rospy.get_param("~publish_viz", True)

        self.world_publisher = WorldPublisher("ar_tracks", self.global_frame_id)

        # self.marker_publisher = MarkerPublisher("ar_markers")

        self.ar_pose_marker_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.observation_callback)
        self.ar_nodes = {}
        self.blacklist_id = []
        self.id_link = {} # Dictionarry for tag ID




    # def camera_info_callback(self, msg):get_worget_worldget_worldld
    #     """ """
    #     if self.camera_info is None:
    #         rospy.loginfo("[ar_perception] Camera info received !")
    #     self.camera_info = msg
    #     self.camera_frame_id = msg.header.frame_id
    #     self.robot_camera = Camera().from_msg(msg,
    #                                           clipnear=self.robot_camera_clipnear,
    #                                           clipfar=self.robot_camera_clipfar)

    def observation_callback(self, ar_marker_msgs):
        """
        """


        # if self.robot_camera is not None or True:
        #
        #     header.stamp = rospy.Time()
        #
        #
        #     # success, view_pose = self.tf_bridge.get_pose_from_tf(self.global_frame_id, self.camera_frame_id)
        #     success=True
        #     if success is not True:
        #         rospy.logwarn("[ar_perception] The camera sensor is not localized in world space (frame '{}'), please check if the sensor frame is published in /tf".format(self.global_frame_id))
        #     else:

        all_nodes = []
        header = ar_marker_msgs.header
        for marker in ar_marker_msgs.markers:

            if not(marker.id in self.blacklist_id):
                if not (marker.id in self.id_link):
                    self.new_node(marker)
                id = self.id_link[marker.id]
                pose = Vector6D().from_msg(marker.pose.pose)
                header = marker.header
                if self.ar_nodes[id].pose is None:
                    self.ar_nodes[id].pose = Vector6DStable(x=pose.pos.x, y=pose.pos.y, z=pose.pos.z,
                                                                   rx=pose.rot.x, ry=pose.rot.y, rz=pose.rot.z, time=header.stamp)
                else:
                    self.ar_nodes[id].pose.pos.update(x=pose.pos.x, y=pose.pos.y, z=pose.pos.z, time=header.stamp)
                    self.ar_nodes[id].pose.rot.update(x=pose.rot.x, y=pose.rot.y, z=pose.rot.z, time=header.stamp)

                all_nodes.append(self.ar_nodes[id])
                # print self.ar_nodes[id].id



            self.world_publisher.publish(all_nodes, [],header)
            # if self.publish_viz is True:
            #     self.marker_publisher.publish(all_nodes, header)

            if self.publish_tf is True:
                self.tf_bridge.publish_tf_frames(all_nodes, [], header)
            # print self.ar_nodes


    def new_node(self,marker):
        #Get real id of marker id from onto
        #get mesh of marker id from onto
        #get label from onto

        node = SceneNode()
        pose = Vector6D().from_msg(marker.pose.pose)
        nodeid = self.onto.individuals.getFrom("hasArId","real#"+str(marker.id))
        # nodeid = self.onto.individuals.getFrom("hasArId","real#230")
        # nodeid = "cube_GBTG_2"
        # print self.onto.individuals.getType("Cube")
        if nodeid == []:
            self.blacklist_id.append(marker.id)
        else:
            self.id_link[marker.id]=nodeid[0]
            path=self.onto.individuals.getOn(nodeid[0],"hasMesh")[0].split("#")[-1]

            node.label ="label"
            shape = Mesh(path,
                         x=0, y=0, z=0,
                         rx=0, ry=0, rz=0)
            shape.color[0] = 0
            shape.color[1] = 0
            shape.color[2] = 0
            shape.color[3] = 1.0
            node.shapes.append(shape)
            node.id = nodeid[0]
            self.ar_nodes[nodeid[0]] = node

    # def send_ar_tag(self,msg):
    #     ret_list = []
    #     for i in msg.ids:
    #         pose_s =PoseStamped()
    #         if not i in self.ar_nodes.keys():
    #             pose_s.header.frame_id=''
    #         else:
    #             node = self.ar_nodes[i]
    #             if node.pose is None:
    #                 pose_s.header.frame_id=''
    #             else:
    #                 pose_s.pose.position.x =node.pose.pos.x
    #                 pose_s.pose.position.y =node.pose.pos.y
    #                 pose_s.pose.position.z =node.pose.pos.z
    #                 quat = quaternion_from_euler(node.pose.rot.x,node.pose.rot.y,node.pose.rot.z)
    #                 pose_s.pose.orientation.x =quat[0]
    #                 pose_s.pose.orientation.y =quat[1]
    #                 pose_s.pose.orientation.z =quat[2]
    #                 pose_s.pose.orientation.w =quat[3]
    #                 print(node)
    #                 pose_s.header.stamp = node.pose.pos.last_update
    #         ret_list.append(pose_s)
    #         ret = GetPoseResponse()
    #         ret.poses=ret_list
    #     return ret

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == "__main__":
    rospy.init_node("ar_perception", anonymous=False)
    perception = ArPerceptionNode().run()
