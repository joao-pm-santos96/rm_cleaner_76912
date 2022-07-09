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
        cleaned_cells = np.sum(np.array(map.data) == COSTMAP_FREE)

        self.cleaned_area = cleaned_cells * (resolution ** 2)

        self.clean_ratio = self.cleaned_area / self.total_area if self.total_area != 0 else 0
        rospy.loginfo(f"Cleaned {self.cleaned_area:.2f} of {self.total_area:.2f} ({self.clean_ratio*100:.2f}%)")

    def config(self):
        # TODO get params        
        self.map_topic = "/map"
        self.cleaned_area_topic = "/cleaned_area/costmap/costmap"
        self.move_base_ns = "move_base"
        self.rate = rospy.Rate(5) #hz

        # Subscribe
        rospy.Subscriber(self.cleaned_area_topic, OccupancyGrid, self.cleaned_map_callback)

        # Action client
        self.move_client = actionlib.SimpleActionClient(self.move_base_ns, MoveBaseAction)

        rospy.loginfo("Bot configured")

    def get_map_details(self):
        rospy.loginfo(f"Waiting for map on topic {self.map_topic}")
        occ_map = rospy.wait_for_message(self.map_topic, OccupancyGrid)
        rospy.loginfo(f"Map received")

        resolution = occ_map.info.resolution

        map_np = np.array(occ_map.data)
        free_cells = np.count_nonzero(map_np == COSTMAP_FREE)
        self.total_area = free_cells * (resolution ** 2)

        rospy.loginfo(f"Total area: {self.total_area} m2")

    def clean(self):
        
        self.move_client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_topic if self.map_topic[0] != "/" else self.map_topic[1:]

        while True:
            goal.target_pose.header.stamp = rospy.Time.now()

            goal.target_pose.pose.position.x = random.uniform(-10, 10)
            goal.target_pose.pose.position.y = random.uniform(-10, 10)
            quat_tf = quaternion_from_euler(0, 0, random.uniform(0,2*3.14))
            quat_msg = Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])
            goal.target_pose.pose.orientation = quat_msg

            self.move_client.send_goal(goal)
            wait = self.move_client.wait_for_result()

            if not wait:
                pass

            self.rate.sleep()


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
        bot.get_map_details()
        bot.clean()

        rospy.spin()

    except Exception as e:
        rospy.logfatal(e)


