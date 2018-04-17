#!/usr/bin/env python

import rospy
from scxml_interpreter import SCXMLInterpreter
#from scxml_interpreter import read_path
from roslib.packages import get_pkg_dir
#from scxml_interpreter import MaxDepth
from xml.etree.ElementTree import TreeBuilder
import smach_ros
from os import path
import os
#RS
RE_PREFIXED_TAG = "$"
RE_PREFIXED_BEGIN_TAG = "${"
RE_PREFIXED_END_TAG = "}"

if __name__ == "__main__":
    rospy.init_node('task_manager')
    #scxml_file = r"/home/administrator/catkin_ws/src/example/resources/scxml/ssm_tutorial.scxml"
    scxml_file = r"/home/administrator/shruti/scxml_interpreter/resources/scxml/wait_skill.scxml"
    #if os.path.splitext(scxml_file)[1] == ".scxml":
    interpreter = SCXMLInterpreter(scxml_file)
    rospy.spin()
