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

    def cleaned_map_callback(self, map):
        resolution = map.info.resolution
        self.map_cleaned = np.asarray(map.data, dtype=np.int8)
        cleaned_cells = np.sum(self.map_cleaned == COSTMAP_FREE)

        self.cleaned_area = cleaned_cells * (resolution ** 2)

        self.clean_ratio = self.cleaned_area / self.total_area if self.total_area != 0 else 0
        rospy.loginfo(f"Cleaned {self.cleaned_area:.2f} of {self.total_area:.2f} ({self.clean_ratio*100:.2f}%)")

    def config(self):
        # TODO get params        
        self.map_topic = "/map"
        self.cleaned_area_topic = "/cleaned_area/costmap/costmap"
        self.move_base_ns = "move_base"
        self.clean_goal = 1.0
        self.rate = rospy.Rate(15) #hz
        self.footprint_w = 0.9 * 4
        self.footprint_h = 0.6

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

        # w_min = 0 + self.map_origin.position.x
        # w_max = self.map_w * self.resolution + self.map_origin.position.x

        # h_min = 0 + self.map_origin.position.y
        # h_max = self.map_h * self.resolution + self.map_origin.position.y

        map_2d = self.map.reshape(self.map_h, self.map_w)
        a = np.where(map_2d == COSTMAP_FREE)

        h_min = np.min(a[0]) * self.resolution + self.map_origin.position.x
        h_max = np.max(a[0]) * self.resolution + self.map_origin.position.x

        w_min = np.min(a[1]) * self.resolution + self.map_origin.position.y
        w_max = np.max(a[1]) * self.resolution + self.map_origin.position.y

        for h in np.arange(h_min, h_max, self.footprint_h):
            ang = np.pi if ang == 0 else 0

            steps = np.arange(w_min, w_max, self.footprint_w) if ang == 0 else np.arange(w_max, w_min, -self.footprint_w)

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




    # def clean_good(self):

    #     rospy.loginfo("Start cleaning")
        
    #     self.move_client.wait_for_server()

    #     goal = MoveBaseGoal()
    #     goal.target_pose.header.frame_id = self.map_topic if self.map_topic[0] != "/" else self.map_topic[1:]

    #     while True:

    #         min_len = np.min([self.map.shape[0], self.map_cleaned.shape[0]])

    #         free_cells = np.where((self.map[0:min_len] == COSTMAP_FREE) & (self.map_cleaned[0:min_len] == COSTMAP_FREE))
    #         free_idxs = np.unravel_index(free_cells, (self.map_w,self.map_h))
            
    #         idx = random.randrange(0, free_idxs[0].shape[1])
            
    #         x = free_idxs[0][0][idx]
    #         x = x * self.resolution + self.map_origin.position.x

    #         y = free_idxs[1][0][idx]
    #         y = y * self.resolution + self.map_origin.position.y

    #         # x = random.uniform(-5,5)
    #         # y = random.uniform(-5,5)

    #         goal.target_pose.pose.position.x = x
    #         goal.target_pose.pose.position.y = y
    #         quat_tf = quaternion_from_euler(0, 0, random.uniform(0,2*3.14))
    #         quat_msg = Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])
    #         goal.target_pose.pose.orientation = quat_msg

    #         self.move_client.send_goal(goal)
    #         wait = self.move_client.wait_for_result()

    #         if not wait:
    #             pass

    #         if self.clean_ratio > 0.95: # TODO
    #             rospy.signal_shutdown("Done")
    
    #         self.rate.sleep()

    # def clean_1(self):

    #     rospy.loginfo("Start cleaning")
        
    #     self.move_client.wait_for_server()

    #     goal = MoveBaseGoal()
    #     goal.target_pose.header.frame_id = self.map_topic if self.map_topic[0] != "/" else self.map_topic[1:]

    #     tfBuffer = tf2_ros.Buffer()
    #     listener = tf2_ros.TransformListener(tfBuffer)

    #     while not rospy.is_shutdown():

    #         try: # TODO params frames
    #             trans = tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time())
    #         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #             self.rate.sleep()
    #             continue

    #         # TODO params
    #         x = random.uniform(-2,2)
    #         y = random.uniform(-2,2)
    #         ang = random.uniform(-3.14, 3.14)
    #         quat_tf = quaternion_from_euler(0, 0, ang)
    #         quat = Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])


    #         goal.target_pose.pose.position.x = x
    #         goal.target_pose.pose.position.y = y
    #         goal.target_pose.pose.orientation = quat
            
    #         goal.target_pose = tf2_geometry_msgs.do_transform_pose(goal.target_pose, trans)

    #         self.move_client.send_goal(goal)
    #         wait = self.move_client.wait_for_result()

    #         if not wait:
    #             pass

    #         if self.clean_ratio > 0.95: # TODO
    #             rospy.signal_shutdown("Done")
    
    #         self.rate.sleep()

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

        rospy.spin()

    except Exception as e:
        rospy.logfatal(e)


