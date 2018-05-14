#!/usr/bin/env python

import rospy
import rospkg
import os
from scxml_interpreter.scxml_parser import SCXMLParser
from scxml_interpreter.skeleton_class import RootStateSkeleton,CompoundStateSkeleton
import unittest
import xml.etree.ElementTree as etree

def parser():

    rospy.init_node('task_manager')
    pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
    scxml_file = os.path.join(pkg_path, "resources/scxml/wait_skill.scxml")
    scxml_parser = SCXMLParser()
    SCXMLSkeleton = scxml_parser.parcing_scxml(scxml_file)
    SCXMLRootSkeleton = scxml_parser.root_skeleton(SCXMLSkeleton)


if __name__ == "__main__":
    interpreter = parser()



