#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import cv2
import rospy
from uwds3_msgs.msg import WorldStamped
from pyuwds3.types.scene_node import SceneNode
from pyuwds3.types.camera import Camera
from sensor_msgs.msg import CameraInfo
from pyuwds3.utils.tf_bridge import TfBridge
from pyuwds3.utils.view_publisher import ViewPublisher
from pyuwds3.utils.marker_publisher import MarkerPublisher
from pyuwds3.utils.world_publisher import WorldPublisher
from pyuwds3.reasoning.simulation.internal_simulator import InternalSimulator
from pyuwds3.reasoning.monitoring.graphic_monitor import GraphicMonitor
from pyuwds3.reasoning.monitoring.perspective_monitor import PerspectiveMonitor
from pyuwds3.reasoning.monitoring.heatmap import Heatmap
from pyuwds3.types.vector.vector6d_stable import Vector6DStable
from pyuwds3.types.shape.mesh import Mesh
from  pr2_motion_tasks_msgs.msg import RobotAction


DEFAULT_SENSOR_QUEUE_SIZE = 3


class InternalSimulatorNode(object):
    """ Standalone node for the internal simulator """
    def __init__(self):
        """ """
        self.tf_bridge = TfBridge()

        self.n_frame = rospy.get_param("~n_frame", 4)
        self.frame_count = 0

        self.robot_camera = None
        self.camera_info = None
        self.camera_frame_id = None

        self.global_frame_id = rospy.get_param("~global_frame_id", "odom")
        self.base_frame_id = rospy.get_param("~base_frame_id", "odom")

        self.use_ar_tags = rospy.get_param("~use_ar_tags", True)
        self.ar_tags_topic = rospy.get_param("ar_tags_topic", "ar_tracks")
        self.ar_tags_topic="ar_tracks"
        self.use_ar_tags = True


        self.use_motion_capture = rospy.get_param("~use_motion_capture", False)
        self.motion_capture_topic = rospy.get_param("~motion_capture_topic", "motion_capture_tracks")
        self.use_motion_capture=True
        self.motion_capture_topic="mocap_tracks"

        use_simulation_gui = rospy.get_param("~use_simulation_gui", True)
        simulation_config_filename = rospy.get_param("~simulation_config_filename", "")
        cad_models_additional_search_path = rospy.get_param("~cad_models_additional_search_path", "")
        static_entities_config_filename = rospy.get_param("~static_entities_config_filename", "")
        robot_urdf_file_path = rospy.get_param("~robot_urdf_file_path", "")
        self.internal_simulator = InternalSimulator(True,
                                                    simulation_config_filename,
                                                    cad_models_additional_search_path,
                                                    static_entities_config_filename,
                                                    robot_urdf_file_path,
                                                    self.global_frame_id,
                                                    self.base_frame_id)
        self.physics_monitor = GraphicMonitor(internal_simulator=self.internal_simulator)
        if self.use_motion_capture is True:
            self.motion_capture_tracks = []
            self.motion_capture_sub = rospy.Subscriber(self.motion_capture_topic, WorldStamped, self.motion_capture_callback, queue_size=DEFAULT_SENSOR_QUEUE_SIZE)

        if self.use_ar_tags is True:
            self.ar_tags_tracks = []
            self.ar_tags_sub = rospy.Subscriber(self.ar_tags_topic, WorldStamped, self.ar_tags_callback, queue_size=DEFAULT_SENSOR_QUEUE_SIZE)


        self.ar_tags_sub = rospy.Subscriber("/mocap_tracks", rospy.AnyMsg, self.publish_view)
        self.pick_subsc = rospy.Subscriber("/pr2_tasks_node/pr2_facts",RobotAction, self.pick_callback)

    def publish_view(self,tf):
        self.physics_monitor.publish_view(tf)

    def pick_callback(self,msg):
        self.physics_monitor.pick_callback(msg)

    def ar_tags_callback(self, world_msg):

        ar_tags_tracks = []
        for node in world_msg.world.scene:
            # print self.internal_simulator.entity_id_map
            ar_tags_tracks.append(SceneNode().from_msg(node))
        self.ar_tags_tracks = ar_tags_tracks
        for i in ar_tags_tracks:
            if i.id=="table_1":
                print i.last_update
        #world_msg.header.frame_id[1:]
        if world_msg.header.frame_id[1:] != '':
            s,pose =self.tf_bridge.get_pose_from_tf(self.global_frame_id, world_msg.header.frame_id[1:])
        else:
            pose=None

        self.physics_monitor.monitor(ar_tags_tracks, pose, world_msg.header)

    def motion_capture_callback(self, world_msg):
        motion_capture_tracks = []
        for node in world_msg.world.scene:
            motion_capture_tracks.append(SceneNode().from_msg(node))
        self.physics_monitor.mocap(motion_capture_tracks,world_msg.header)

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == "__main__":
    rospy.init_node("internal_simulator", anonymous=False)
    perception = InternalSimulatorNode().run()
