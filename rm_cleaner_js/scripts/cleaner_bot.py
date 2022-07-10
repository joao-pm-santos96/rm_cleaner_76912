#!/usr/bin/env python3
"""
***DESCRIPTION***
"""

"""
IMPORTS
"""
import rospy
import actionlib
import numpy as np

import tf2_ros
import tf2_geometry_msgs
import random

from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion

from tf.transformations import quaternion_from_euler

"""
METADATA
"""
__author__ = 'Joao Santos'
__copyright__ = 'Copyright July2022'
__credits__ = ['Joao Santos']
__version__ = '1.0.0'
__maintainer__ = 'Joao Santos'
__email__ = 'joao.pm.santos96@gmail.com'
__status__ = 'Production'

"""
TODO
"""

"""
GLOBALS
"""
COSTMAP_OCCUP = 100
COSTMAP_FREE = 0
COSTMAP_UNKN = 255

"""
CLASS DEFINITIONS
"""
class CleanerBot:

    def __init__(self):        
        rospy.init_node('cleaner_bot')
        self.total_area = 0
        self.cleaned_area = 0
        self.clean_ratio = 0

        rospy.on_shutdown(self.on_shutdown)

    def cleaned_map_callback(self, map):
        resolution = map.info.resolution
        self.map_cleaned = np.asarray(map.data, dtype=np.int8)
        cleaned_cells = np.sum(self.map_cleaned == COSTMAP_FREE)

        self.cleaned_area = cleaned_cells * (resolution ** 2)

        self.clean_ratio = self.cleaned_area / self.total_area if self.total_area != 0 else 0
        rospy.loginfo(f"Cleaned {self.cleaned_area:.2f} of {self.total_area:.2f} ({self.clean_ratio*100:.2f}%)")

    def config(self):
        # # TODO get params        
        # self.map_topic = "/map"
        # self.cleaned_area_topic = "/cleaned_area/costmap/costmap"
        # self.move_base_ns = "move_base"
        # self.clean_goal = 1.0
        # self.rate = rospy.Rate(15) #hz
        # self.footprint_w = 0.9 * 4
        # self.footprint_h = 0.6


        # Get params
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.cleaned_area_topic = rospy.get_param("~cleaned_area_topic", "costmap")
        self.move_base_ns = rospy.get_param("~move_base_ns", "move_base")
        self.clean_goal = rospy.get_param("~clean_goal", 1.0)
        self.delta_w = rospy.get_param("~delta_w", 1.0)
        self.delta_h = rospy.get_param("~delta_h", 0.5)
        frequency = rospy.get_param("~frequency", 15)
        self.rate = rospy.Rate(frequency)

        # Subscribe
        rospy.Subscriber(self.cleaned_area_topic, OccupancyGrid, self.cleaned_map_callback)

        # Action client
        self.move_client = actionlib.SimpleActionClient(self.move_base_ns, MoveBaseAction)

        rospy.loginfo("Bot configured")

    def get_infos(self):

        rospy.loginfo(f"Waiting for map on topic {self.map_topic}")

        occ_map = rospy.wait_for_message(self.map_topic, OccupancyGrid)
        
        rospy.loginfo(f"Map received")

        self.resolution = occ_map.info.resolution
        self.map_w  = occ_map.info.width
        self.map_h  = occ_map.info.height
        self.map_origin = occ_map.info.origin

        self.map = np.asarray(occ_map.data, dtype=np.int8)#.reshape(occ_map.info.height, occ_map.info.width)
        free_cells = np.count_nonzero(self.map == COSTMAP_FREE)
        self.total_area = free_cells * (self.resolution ** 2)

        rospy.loginfo(f"Total area: {self.total_area} m2")


    def clean(self):

        rospy.loginfo("Start cleaning")
        
        self.move_client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_topic if self.map_topic[0] != "/" else self.map_topic[1:]

        ang = None

        map_2d = self.map.reshape(self.map_h, self.map_w)
        a = np.where(map_2d == COSTMAP_FREE)

        h_min = np.min(a[0]) * self.resolution + self.map_origin.position.x
        h_max = np.max(a[0]) * self.resolution + self.map_origin.position.x
        h_min = h_min + self.delta_h / 2
        h_max = h_max - self.delta_h / 2

        w_min = np.min(a[1]) * self.resolution + self.map_origin.position.y
        w_max = np.max(a[1]) * self.resolution + self.map_origin.position.y
        w_min = w_min + self.delta_w / 2
        w_max = w_max - self.delta_w / 2

        for h in np.arange(h_min, h_max, self.delta_h):
            ang = np.pi if ang == 0 else 0

            steps = np.arange(w_min, w_max, self.delta_w) if ang == 0 else np.arange(w_max, w_min, -self.delta_w)

            for w in steps:

                goal.target_pose.pose.position.x = w
                goal.target_pose.pose.position.y = h
                quat_tf = quaternion_from_euler(0, 0, ang)
                quat_msg = Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])
                goal.target_pose.pose.orientation = quat_msg

                self.move_client.send_goal(goal)
                wait = self.move_client.wait_for_result()

                if not wait:
                    rospy.loginfo("Goal not achieved")

                if self.clean_ratio > self.clean_goal:
                    rospy.signal_shutdown("Done")
                    
                self.rate.sleep()

    def on_shutdown(self):
        rospy.loginfo("Clean cycle ended")
        rospy.loginfo(f"Cleaned {self.clean_ratio*100:.2f}%")


"""
FUNCTIONS DEFINITIONS
"""

"""
MAIN
"""
if __name__ == '__main__':
    
    
    try:
        bot = CleanerBot()
        bot.config()
        bot.get_infos()
        bot.clean()

        # rospy.spin()

    except Exception as e:
        rospy.logfatal(e)


