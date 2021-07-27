#!/usr/bin/env python
"""
Simply tells the mapper to load a .panmap file as soon as possible.
"""
import rospy
from panoptic_mapping_msgs.srv import SaveLoadMap

if __name__ == '__main__':
    rospy.init_node('map_loader', anonymous=True)
    path = rospy.get_param('~path', "")
    srv_name = rospy.get_param('~srv_name', "/panoptic_mapper/load_map")
    delay = rospy.get_param('~delay', 0)
    rospy.loginfo("map_loader: Waiting for service '%s'." % srv_name)
    rospy.wait_for_service(srv_name)
    rospy.sleep(delay)
    srv = rospy.ServiceProxy(srv_name, SaveLoadMap)
    srv(path)
    rospy.loginfo("[map_loader] Finished after calling service.")
