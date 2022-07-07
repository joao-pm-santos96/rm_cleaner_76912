#!/usr/bin/env python3
"""
***DESCRIPTION***
"""

"""
IMPORTS
"""
import rospy
import numpy as np

from nav_msgs.msg import OccupancyGrid
# from map_msgs.msg import OccupancyGridUpdate

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

    def area_map_callback(self, map):
        resolution = map.info.resolution
        cleaned_cells = np.sum(np.array(map.data) == COSTMAP_FREE)

        self.cleaned_area = cleaned_cells * (resolution ** 2)

        self.clean_ratio = self.cleaned_area / self.total_area if self.total_area != 0 else 0

    def config(self):
        # TODO get params        
        self.map_topic = "/map"
        self.cleaned_area_topic = "/cleaned_area/costmap/costmap"

        # Subscribe
        rospy.Subscriber(self.cleaned_area_topic, OccupancyGrid, self.area_map_callback)

        rospy.loginfo("Bot configured")

    def get_map_details(self):
        rospy.loginfo(f"Waiting for map on topic {self.map_topic}")
        occ_map = rospy.wait_for_message(self.map_topic, OccupancyGrid)
        rospy.loginfo(f"Map received")

        resolution = occ_map.info.resolution
        # map_w  = occ_map.info.width
        # map_h  = occ_map.info.height
        # map_origin = occ_map.info.origin

        map_np = np.array(occ_map.data)
        free_cells = np.count_nonzero(map_np == COSTMAP_FREE)
        self.total_area = free_cells * (resolution ** 2)

        rospy.loginfo(f"Total area: {self.total_area} m2")


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


        rospy.spin()

    except Exception as e:
        rospy.logfatal(e)


