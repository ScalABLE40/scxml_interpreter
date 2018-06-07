#!/usr/bin/env python

from scxml_interpreter.smach_builder import SmachBuilder
#from scxml_interpreter.scxml_parser import SmachBuilder
from std_msgs.msg import Int8

class SCXMLManager():
    def __init__(self):
        self._parsing_file         = rospy.Service("~srv/parsing",MSGTYPE,self._parsing_file_cb)
        self._create_state_machine = rospy.Service("~srv/create_state_machine", MSGTYPE, self._create_state_machine_cb)
        self._current_status_pub   = rospy.Publisher("~status",Int8) 
       
        