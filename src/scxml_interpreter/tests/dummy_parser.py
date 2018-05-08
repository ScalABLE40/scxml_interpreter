#!/usr/bin/env python

import rospy
import rospkg
import os
from scxml_interpreter.scxml_parser import SCXMLInterpreter


if __name__ == "__main__":
    rospy.init_node('task_manager')

    pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
    scxml_file = os.path.join(pkg_path, "resources/scxml/wait_skill.scxml")
    #if os.path.splitext(scxml_file)[1] == ".scxml":
    interpreter = SCXMLInterpreter(scxml_file)
            
