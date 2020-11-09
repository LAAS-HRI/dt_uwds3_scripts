#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import cv2
import numpy as np
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
from tf.transformations import euler_matrix

DEFAULT_SENSOR_QUEUE_SIZE = 1
VALUEY = 0.3
VALUEZ = 0.3

class ArPerceptionNode(object):
    def __init__(self):
        """
        """
        self.tf_bridge = TfBridge()


        # ontologiesManipulator =OntologiesManipulator()
        # self.onto = ontologiesManipulator.get("robot")

        print("hhhhhh")
        self.global_frame_id = rospy.get_param("~global_frame_id")
        print self.global_frame_id
        print("hhhhhh")
        self.ontologies_manip = OntologiesManipulator()
        self.ontologies_manip.add("robot")
        self.onto=self.ontologies_manip.get("robot")
        self.onto.close()
        print("j")
        self.global_frame_id = rospy.get_param("~global_frame_id", "map")
        self.publish_tf = rospy.get_param("~publish_tf", False)


        self.world_publisher = WorldPublisher("ar_tracks", self.global_frame_id)

        self.ar_pose_marker_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.observation_callback)
        self.ar_pose_visible_marker = rospy.Subscriber("ar_pose_visible_marker",AlvarVisibleMarkers,self.visible_observation_callback)
        self.joint_state_subscriber = rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
        self.ar_nodes = {}
        self.blacklist_id = []
        self.id_link = {} # Dictionarry for tag ID



    def observation_callback(self, ar_marker_msgs):
        """
        """

        all_nodes = []
        header = ar_marker_msgs.header

        for marker in ar_marker_msgs.markers:
            if not(marker.id in self.blacklist_id):
                if not (marker.id in self.id_link):
                    self.new_node(marker)
                # print marker.id
                # print self.id_link
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


            self.world_publisher.publish(self.ar_nodes.values(), [],header)
            # print("pub")

            if self.publish_tf is True:
                self.tf_bridge.publish_tf_frames(self.ar_nodes.values(), [], header)
            # print self.ar_nodes


    def validity(self,marker):
        head_pose = self.get_pose_from_tf( marker.header.frame_id,
                                           "head_tilt_joint",
                                            marker.header.stamp)


        x =  marker.pose.pose.position.x
        y =  marker.pose.pose.position.y
        z =  marker.pose.pose.position.z
        mpose=np.array([x,y,z])
        direction = np.array([x-head_pose.pos.x,y-head_pose.pos.y,z-head_pose.pos.z])

        rotation_matrix = euler_matrix(head_pose.rot.x,
                                                            head_pose.rot.y,
                                                            head_pose.rot.z)
        un_homogen = lambda x: np.array([i/(x[-1]*1.) for i in x[:-1]])
        xaxis = un_homogen(np.dot(rotation_matrix,[1,0,0,1]))
        yaxis = un_homogen(np.dot(rotation_matrix,[0,1,0,1]))
        zaxis = un_homogen(np.dot(rotation_matrix,[0,0,1,1]))

        proj_on_y=np.dot(direction,yaxis)*yaxis/(np.linalg.norm(yaxis))
        proj_on_z=np.dot(direction,zaxis)*zaxis/(np.linalg.norm(zaxis))
        proj_on_xz_plane = direction-proj_on_y
        proj_on_xy_plane = direction-proj_on_z
        dot_x_xy =np.dot(proj_on_xy_plane,xaxis)/np.linalg.norm(xaxis)/np.linalg.norm(proj_on_xy_plane)
        dot_x_xz =np.dot(proj_on_xz_plane,xaxis)/np.linalg.norm(xaxis)/np.linalg.norm(proj_on_xz_plane)
        if abs(dot_x_xy)<VALUEZ and abs(dot_x_xz)<VALUEY:
            return True




    def visible_observation_callback(self, ar_marker_msgs):
        """
        """
        dic_marker = {}
        res_list=[]
        for marker in ar_marker_msgs:
            if self.validity.marker(marker):
                if marker.main_id in dic_marker.keys():
                    dic_marker[marker.main_id].append(maker)
                else:
                    dic_marker[marker.main_id]=[marker]

        for marker_main_id in dic_marker.keys():
            ret =dic_marker[marker_main_id][0]
            ret_conf = ret.confidence
            for marker in dic_marker[marker_main_id]:
                if marker.confidence>ret_conf:
                    ret_conf = marker.confidence
                    ret = marker
            res_list.append(ret)

        self.observation(res_list,ar_marker_msgs.header)


    def observation(self, ar_marker_list,header):
        """
        """

        all_nodes = []

        for marker in ar_marker_list:
            if not(marker.main_id in self.blacklist_id):
                if not (marker.main_id in self.id_link):
                    self.new_node(marker)
                # print marker.main_id
                # print self.id_link
                id = self.id_link[marker.main_id]
                pose = Vector6D().from_msg(marker.pose.pose)
                header = marker.header
                if self.ar_nodes[id].pose is None:
                    self.ar_nodes[id].pose = Vector6DStable(x=pose.pos.x, y=pose.pos.y, z=pose.pos.z,
                                                                   rx=pose.rot.x, ry=pose.rot.y, rz=pose.rot.z, time=header.stamp)
                else:
                    self.ar_nodes[id].pose.pos.update(x=pose.pos.x, y=pose.pos.y, z=pose.pos.z, time=header.stamp)
                    self.ar_nodes[id].pose.rot.update(x=pose.rot.x, y=pose.rot.y, z=pose.rot.z, time=header.stamp)

                all_nodes.append(self.ar_nodes[id])


            self.world_publisher.publish(self.ar_nodes.values(), [],header)
            # print("pub")

            if self.publish_tf is True:
                self.tf_bridge.publish_tf_frames(self.ar_nodes.values(), [], header)
            # print self.ar_nodes




    def new_node(self,marker):
        #Get real id of marker id from onto
        #get mesh of marker id from onto
        #get label from onto

        node = SceneNode()
        pose = Vector6D().from_msg(marker.pose.pose)
        nodeid = self.onto.individuals.getFrom("hasArId","real#"+str(marker.id))
        # print nodeid
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

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == "__main__":
    rospy.init_node("ar_perception", anonymous=False)
    perception = ArPerceptionNode().run()
