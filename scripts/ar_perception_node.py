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
from pyuwds3.types.shape.shape import Shape, ShapeType
from pyuwds3.types.shape.box import Box
from pyuwds3.types.shape.cylinder import Cylinder
from uwds3_msgs.msg import WorldStamped
from pyuwds3.utils.tf_bridge import TfBridge
from pyuwds3.utils.view_publisher import ViewPublisher
from pyuwds3.utils.marker_publisher import MarkerPublisher
from pyuwds3.utils.world_publisher import WorldPublisher
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarVisibleMarkers
import yaml
from geometry_msgs.msg import PoseStamped
from ontologenius import OntologiesManipulator
from ontologenius import OntologyManipulator
from pr2_motion_tasks_msgs.srv import GetPose
from pr2_motion_tasks_msgs.srv import GetPoseResponse
from tf.transformations import euler_matrix
import message_filters
import tf


DEFAULT_SENSOR_QUEUE_SIZE = 1

# FILTERING_Y = 53
# FILTERING_Z = 53
FILTERING_Y = 15
FILTERING_Z = 20
MIN_VEL = 0.001
MIN_ANG = 0.001

class ArPerceptionNode(object):
    def __init__(self):
        """
        """
        self.tf_bridge = TfBridge()

        self.listener = tf.TransformListener()



        # ontologiesManipulator =OntologiesManipulator()
        # self.onto = ontologiesManipulator.get("robot")

        self.global_frame_id = rospy.get_param("~global_frame_id")
        print "global fame id is : " + str(self.global_frame_id)
        self.ontologies_manip = OntologiesManipulator()
        self.ontologies_manip.add("robot")
        self.onto=self.ontologies_manip.get("robot")
        self.onto.close()
        self.global_frame_id = rospy.get_param("~global_frame_id", "map")
        self.publish_tf = rospy.get_param("~publish_tf", False)


        self.world_publisher_global = WorldPublisher("ar_tracks", self.global_frame_id)
        self.world_publisher_local =  WorldPublisher("ar_tracks_local")
        self.marker_publisher = MarkerPublisher("ar_perception_marker2")
        # self.ar_pose_marker_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.observation_callback)

        self.pose_marker_sub = message_filters.Subscriber("ar_pose_marker", AlvarMarkers)
        self.visible_marker_sub = message_filters.Subscriber("ar_pose_visible_marker",AlvarVisibleMarkers)

        self.synchronous_marker_sub = message_filters.TimeSynchronizer([self.visible_marker_sub,self.pose_marker_sub], 10)
        self.synchronous_marker_sub.registerCallback(self.visible_observation_callback)
        self.filtering_y_axis = rospy.get_param("~filtering_y_axis", FILTERING_Y)
        self.filtering_z_axis = rospy.get_param("~filtering_z_axis", FILTERING_Z)
        self.minimum_velocity = rospy.get_param("~minimum_velocity", MIN_VEL)
        self.minimum_angular_velocity = rospy.get_param("~minimum_angular_velocity", MIN_ANG)
        self.ar_nodes = {}
        self.ar_nodes_local={}

        self.blacklist_id = []
        self.id_link = {} # Dictionarry for tag ID
        # self.joint_state_subscriber = rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
        self.last_head_pose = None
        self.last_time_head_pose = rospy.Time(0)
        print "filtering is " + str(self.filtering_y_axis) +"° x " +str(self.filtering_z_axis)+"°"
        # shp1 = Box(2,0.01,1, "shp1",x=0,y=0,z=0,r=1.,a=0,rz=np.radians(self.filtering_y_axis))
        # shp2 = Box(2,0.01,1, "shp2",y=0,x=0,z=0,r=1.,a=0,rz=-np.radians(self.filtering_y_axis))
        # shp3 = Box(2,1,0.01, "shp3",x=0,z=0,y=0,b=1.,a=0,ry=np.radians(self.filtering_z_axis))
        # shp4 = Box(2,1,0.01, "shp4",x=0,y=0,z=0,b=1.,a=0,ry=-np.radians(self.filtering_z_axis))
        # sn1 = SceneNode(pose = Vector6D(x=0,y=0,z=0,rx=0,ry=0,rz=0),label="no_fact")
        # sn2 = SceneNode(pose = Vector6D(x=0,y=0,z=0,rx=0,ry=0,rz=0),label="no_fact")
        # sn3 = SceneNode(pose = Vector6D(x=0,y=0,z=0,rx=0,ry=0,rz=0),label="no_fact")
        # sn4 = SceneNode(pose = Vector6D(x=0,y=0,z=0,rx=0,ry=0,rz=0),label="no_fact")
        # sn1.id="sn1"
        # sn2.id="sn2"
        # sn3.id="sn3"
        # sn4.id="sn4"
        # sn1.shapes=[shp1]
        # sn2.shapes=[shp2]
        # sn3.shapes=[shp3]
        # sn4.shapes=[shp4]
        # self.ar_nodes["sn1"]=sn1
        # self.ar_nodes["sn2"]=sn2
        # self.ar_nodes["sn3"]=sn3
        # self.ar_nodes["sn4"]=sn4
    def observation_callback(self, ar_marker_msgs):
        """
        """

        all_nodes = []
        header = ar_marker_msgs.header

        for marker in ar_marker_msgs.markers:
            print marker.id
            if not(marker.id in self.blacklist_id):
                if not (marker.id in self.id_link):
                    self.new_node(marker)
                print marker.id
                # print self.id_link

                # print self.id_link.keys()
                id = self.id_link[marker.id]
                pose = Vector6D().from_msg(marker.pose.pose)
                header = marker.header
                if self.ar_nodes[id].pose is None:
                    self.ar_nodes[id].pose = Vector6DStable(x=pose.pos.x, y=pose.pos.y, z=pose.pos.z,
                                                                   rx=pose.rot.x, ry=pose.rot.y, rz=pose.rot.z, time=header.stamp)
                else:
                    self.ar_nodes[id].pose.pos.update_no_kalmann(x=pose.pos.x, y=pose.pos.y, z=pose.pos.z, time=header.stamp)
                    self.ar_nodes[id].pose.rot.update_no_kalmann(x=pose.rot.x, y=pose.rot.y, z=pose.rot.z, time=header.stamp)

                all_nodes.append(self.ar_nodes[id])



        self.world_publisher.publish(self.ar_nodes.values(), [],header)
        # print("pub")

        if self.publish_tf is True:
            self.tf_bridge.publish_tf_frames(self.ar_nodes.values(), [], header)
        # print self.ar_nodes




    def movement_validity(self,header):

        # frame_id = header.frame_id
        # if frame_id[0]=='/':
        #     frame_id = frame_id[1:]
        frame_id = "base_footprint"
# "head_mount_kinect2_rgb_optical_frame"
        bool_,head_pose = self.tf_bridge.get_pose_from_tf(frame_id ,
                                                   "head_mount_kinect2_rgb_link",
                                                            header.stamp)
        # self.ar_nodes["sn1"].pose= Vector6DStable(x=head_pose.pos.x, y=head_pose.pos.y, z=head_pose.pos.z,
        #                                                rx=head_pose.rot.x, ry=head_pose.rot.y, rz=head_pose.rot.z, time=header.stamp)
        # self.ar_nodes["sn2"].pose= Vector6DStable(x=head_pose.pos.x, y=head_pose.pos.y, z=head_pose.pos.z,
        #                                                rx=head_pose.rot.x, ry=head_pose.rot.y, rz=head_pose.rot.z, time=header.stamp)
        # self.ar_nodes["sn3"].pose= Vector6DStable(x=head_pose.pos.x, y=head_pose.pos.y, z=head_pose.pos.z,
        #                                                rx=head_pose.rot.x, ry=head_pose.rot.y, rz=head_pose.rot.z, time=header.stamp)
        # self.ar_nodes["sn4"].pose= Vector6DStable(x=head_pose.pos.x, y=head_pose.pos.y, z=head_pose.pos.z,
        #                                                rx=head_pose.rot.x, ry=head_pose.rot.y, rz=head_pose.rot.z, time=header.stamp)
        #init for the first time
        if self.last_head_pose == None:
            self.last_head_pose = head_pose

        vel_movement = 0
        ang_movement = 0
        delta= header.stamp-self.last_time_head_pose
        #If we are on a different time frame : (the head might have moved)
        if header.stamp != self.last_time_head_pose:
            vel_movement = np.linalg.norm(
                                    head_pose.pos.to_array() -
                                    self.last_head_pose.pos.to_array() )/delta.to_sec()
            ang_movement = np.linalg.norm(
                                    head_pose.rot.to_array() -
                                    self.last_head_pose.rot.to_array() )/delta.to_sec()



            self.last_time_head_pose = header.stamp
            self.last_head_pose = head_pose
        return ((vel_movement> self.minimum_velocity) or
          ang_movement> self.minimum_angular_velocity)



    def pos_validity_marker(self,marker):
        header = marker.header
        x =  marker.pose.pose.position.x
        y =  marker.pose.pose.position.y
        z =  marker.pose.pose.position.z
        mpose=np.array([x,y,z])
        return self.pos_validityv2(mpose,header)

    def pos_validityv2(self,mvect,header):
        mpose = Vector6DStable(mvect[0],mvect[1],mvect[2])
        frame_id = header.frame_id
        if frame_id[0]=='/':
            frame_id = frame_id[1:]
        bool_,head_pose = self.tf_bridge.get_pose_from_tf("head_mount_kinect2_rgb_link" ,
                                          frame_id,header.stamp)

        #init for the first time
        if self.last_head_pose == None and bool_:
            self.last_head_pose = head_pose

        mpose.from_transform(np.dot(head_pose.transform(),mpose.transform()))
        #mpose is now in the head frame
        if mpose.pos.x==0:
            return False
        xy_angle = np.degrees(np.arctan(mpose.pos.y/mpose.pos.x))
        xz_angle = np.degrees(np.arctan(mpose.pos.z/mpose.pos.x))

        return (abs(xy_angle)<self.filtering_y_axis and
               abs(xz_angle)<self.filtering_z_axis)



    def pos_validity(self,mpose,header):

        frame_id = header.frame_id
        if frame_id[0]=='/':
            frame_id = frame_id[1:]

        x =  mpose[0]
        y =  mpose[1]
        z =  mpose[2]
        bool_,head_pose = self.tf_bridge.get_pose_from_tf(frame_id ,
                                           "head_mount_kinect2_rgb_link",
                                                    header.stamp)

        #init for the first time
        if self.last_head_pose == None:
            self.last_head_pose = head_pose

        #If the head is moving, it's no time to compute thing, we dont want the frame

        #NOW  we compute the direction


        direction = np.array([x-self.last_head_pose.pos.x,y-self.last_head_pose.pos.y,z-self.last_head_pose.pos.z])

        rotation_matrix = euler_matrix(self.last_head_pose.rot.x,
                                                            self.last_head_pose.rot.y,
                                                            self.last_head_pose.rot.z)
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

        return (abs(dot_x_xy)>np.cos(np.radians(self.filtering_y_axis)) and
               abs(dot_x_xz)>np.cos(np.radians(self.filtering_z_axis)))





    def visible_observation_callback(self,visible_ar_marker_msgs, ar_marker_msgs):
        """
        """
        marker_blacklist=[]
        marker_seen_map={}
        if self.movement_validity(ar_marker_msgs.header):
            self.observation(visible_ar_marker_msgs.header,[],[],{},is_mov=True)
            return

        for marker in visible_ar_marker_msgs.markers:
            # if marker.header.frame_id!='':
            if not self.pos_validity_marker(marker):
                if not marker.main_id in marker_blacklist:
                    marker_blacklist.append(marker.main_id)
        for marker in visible_ar_marker_msgs.markers:
                if (not marker.main_id in marker_blacklist) :
                    if(not marker.main_id in marker_seen_map):
                        marker_seen_map[marker.main_id]=[]
                    marker_seen_map[marker.main_id].append(marker)

        self.observation( visible_ar_marker_msgs.header,ar_marker_msgs.markers,marker_blacklist,marker_seen_map,is_mov=False)


    def observation(self,header_, ar_marker_list,marker_blacklist,marker_seen_map,is_mov=False):
        """
        """
        header = header_
        header_global = rospy.Header(header_.seq,header_.stamp,'/'+self.global_frame_id)
        all_nodes = []
        for marker in ar_marker_list:
            # print marker.id

            if (not((marker.id in self.blacklist_id) or (marker.id in marker_blacklist))):
                if not (marker.id in self.id_link):
                    self.new_node(marker)
            if (not((marker.id in self.blacklist_id) or (marker.id in marker_blacklist))):
                id = self.id_link[marker.id]
                pose = Vector6D().from_msg(marker.pose.pose)
                header = marker.header
                header_global.seq = header.seq
                header_global.stamp = header.stamp
                s,pose_map =self.tf_bridge.get_pose_from_tf(self.global_frame_id, header.frame_id[1:],header.stamp)
                if self.ar_nodes_local[id].pose is None:
                    self.ar_nodes_local[id].pose = Vector6DStable(x=pose.pos.x, y=pose.pos.y, z=pose.pos.z,
                                                                   rx=pose.rot.x, ry=pose.rot.y, rz=pose.rot.z, time=header.stamp)
                if s:
                    if self.ar_nodes[id].pose is None:
                        self.ar_nodes[id].pose = Vector6DStable(x=pose.pos.x, y=pose.pos.y, z=pose.pos.z,
                                                                      rx=pose.rot.x, ry=pose.rot.y, rz=pose.rot.z, time=header.stamp)
                    else:
                            self.ar_nodes[id].pose.pos.update_no_kalmann(x=pose.pos.x, y=pose.pos.y, z=pose.pos.z, time=header.stamp)
                            self.ar_nodes[id].pose.rot.update_no_kalmann(x=pose.rot.x, y=pose.rot.y, z=pose.rot.z, time=header.stamp)
                    for marker_seen in marker_seen_map[marker.id]:
                        pose_s = Vector6D().from_msg(marker_seen.pose.pose)
                        if not marker_seen.id in self.ar_nodes[id].last_seen_position:
                            self.ar_nodes[id].last_seen_position[marker_seen.id] = Vector6DStable(
                            x=pose_s.pos.x, y=pose_s.pos.y, z=pose_s.pos.z,rx=pose_s.rot.x,
                             ry=pose_s.rot.y, rz=pose_s.rot.z, time=header.stamp)
                        else:
                            self.ar_nodes[id].last_seen_position[marker_seen.id].pos.update_no_kalmann(x=pose_s.pos.x, y=pose_s.pos.y, z=pose_s.pos.z, time=header.stamp)
                            self.ar_nodes[id].last_seen_position[marker_seen.id].rot.update_no_kalmann(x=pose_s.rot.x, y=pose_s.rot.y, z=pose_s.rot.z, time=header.stamp)
                        self.ar_nodes[id].last_seen_position[marker_seen.id].from_transform(np.dot(pose_map.transform(),
                                self.ar_nodes[id].last_seen_position[marker_seen.id].transform()))

                    self.ar_nodes[id].pose.from_transform(np.dot(pose_map.transform(),self.ar_nodes[id].pose.transform()))
                    self.ar_nodes[id].last_update = marker.header.stamp
                    print marker.header.stamp.to_sec()
                    print self.ar_nodes[id].last_update.to_sec()


        # print self.ar_nodes.keys()



        self.world_publisher_global.publish(self.ar_nodes.values(), [],header_global)
        self.world_publisher_local.publish(self.ar_nodes_local.values(),[],header)

        if self.publish_tf is True and len(header_global.frame_id)>0:
            self.tf_bridge.publish_tf_frames(self.ar_nodes.values(), [], header_global)
        self.marker_publisher.publish(self.ar_nodes.values(),header_global)
        # print self.ar_nodes




    def new_node(self,marker):
        #Get real id of marker id from onto
        #get mesh of marker id from onto
        #get label from onto
        node = SceneNode()
        node_local=SceneNode()
        nodeid = self.onto.individuals.getFrom("hasArId","real#"+str(marker.id))
        # print n   odeid
        # print nodeid
        # nodeid = self.onto.individuals.getFrom("hasArId","real#230")
        # nodeid = "cube_GBTG_2"
        # print self.onto.individuals.getType("Cube")

        if nodeid == []:
            self.blacklist_id.append(marker.id)
        else:
            # print "hh"
            # print marker.id
            self.id_link[marker.id]=nodeid[0]
            path=self.onto.individuals.getOn(nodeid[0],"hasMesh")[0].split("#")[-1]

            node.label ="label"
            node_local.label = "label"
            print path
            shape = Mesh(path,
                         x=0, y=0, z=0,
                         rx=0, ry=0, rz=0)
            r,g,b=0,0,0
            if "ox" in nodeid[0]:
                r,g,b=0.82,0.42, 0.12
            if "cube" in nodeid[0]:
                r,g,b=0,0,1
            if "GBTB" in nodeid[0]:
                r,g,b= 1,0,0
            if "BGTG" in nodeid[0]:
                r,g,b=0,1,0


            shape.color[0] = r
            shape.color[1] = g
            shape.color[2] = b
            shape.color[3] = 1
            node.shapes.append(shape)
            node.id = nodeid[0]
            node_local.shapes.append(shape)
            node_local.id = nodeid[0]

            self.ar_nodes[nodeid[0]] = node
            self.ar_nodes_local[nodeid[0]] = node_local

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == "__main__":
    rospy.init_node("ar_perception", anonymous=False)
    perception = ArPerceptionNode().run()
