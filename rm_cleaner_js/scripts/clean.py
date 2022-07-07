#!/usr/bin/env python3
import rospy
import actionlib
import random
import numpy as np

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler

def movebase_client(x,y):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # goal.target_pose.pose.position.x = random.uniform(-5,5)
    # goal.target_pose.pose.position.y = random.uniform(-5,5)
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    
    quat_tf = quaternion_from_euler(0, 0, random.uniform(0,2*3.14))
    quat_msg = Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])
    goal.target_pose.pose.orientation = quat_msg



    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':

    map_topic = "/map"

    try:
        rospy.init_node('cleaner')
        
        rospy.loginfo(f"Waiting for map on topic {map_topic}")
        occ_map = rospy.wait_for_message(map_topic, OccupancyGrid)
        
        map_resolution = occ_map.info.resolution
        map_w  = occ_map.info.width
        map_h  = occ_map.info.height
        map_origin = occ_map.info.origin

        map_np = np.array(occ_map.data)
        map_np = map_np.reshape((map_w, map_h), order="F")

        free_cells = np.where(map_np == 0)
        free_area = free_cells[0].shape[0] * (map_resolution ** 2) # m^2


        exit()
        
        
        
        
        # xc = free_cells[0][-1]
        # yc = free_cells[1][-1]

        # xc = xc * map_resolution + map_origin.position.x
        # yc = yc * map_resolution + map_origin.position.y
        # print(xc)
        # print(yc)

        result = movebase_client(xc,yc)
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")