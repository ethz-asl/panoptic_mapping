#!/usr/bin/env python3
"""
Simply tells the mapper to load a .panmap file as soon as possible.
"""
import rospy
from std_srvs.srv import Empty
from panoptic_mapping_msgs.srv import SaveLoadMap

if __name__ == '__main__':
    rospy.init_node('map_loader', anonymous=True)
    path = rospy.get_param('~path', "")
    map_srv_name = rospy.get_param('~map_srv_name',
                                   "/panoptic_mapper/load_map")
    play_srv_name = rospy.get_param('~play_srv_name',
                                    "/flat_data_player/start")
    delay = rospy.get_param('~delay', 0)
    rospy.loginfo("map_loader: Waiting for service '%s'." % map_srv_name)
    rospy.wait_for_service(map_srv_name)
    rospy.sleep(delay)
    # Load the maps.
    srv = rospy.ServiceProxy(map_srv_name, SaveLoadMap)
    srv(path)
    # Start playing the data.
    rospy.sleep(0.5)
    rospy.loginfo("map_loader: Waiting for service '%s'." % play_srv_name)
    rospy.wait_for_service(play_srv_name)
    srv = rospy.ServiceProxy(play_srv_name, Empty)
    srv()
    rospy.loginfo("[map_loader] Finished after calling service.")
