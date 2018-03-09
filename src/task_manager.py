#!/usr/bin/env python

import rospy
from scxml_interpreter import SCXMLInterpreter
import smach_ros

if __name__ == "__main__":
    rospy.init_node('task_manager')

    scxml_file = open('/home/tiago/catkin_ws/src/task_manager_scxml/scxml_interpreter/resources/scxml/test.scxml')

    interpreter = SCXMLInterpreter(scxml_file)
    sm = interpreter.convertSCXML()

    sis = smach_ros.IntrospectionServer('server_name', sm, 'test')

    sis.start()
    sm.execute()

    rospy.spin()
