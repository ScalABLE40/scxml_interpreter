#!/usr/bin/env python
'''
import rospy
import rospkg
import os
from scxml_interpreter.scxml_parser import SCXMLParser
from scxml_interpreter.interfaces import *
import unittest

class test_cases(unittest.TestCase):


    #testing the state of root of state machine
    def test_scxmlstate(self):

        pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
        scxml_file = os.path.join(pkg_path, "resources/scxml/simple_file.scxml")
        scxml_parser = SCXMLParser()
        SCXMLInterface = scxml_parser.parsing_scxml(scxml_file)
        self.assertEqual(str(SCXMLInterface), str(SCXMLRootInterface({}, ["Final_4"], ['WaitSkill'], 'WaitSkill')))

if __name__ == "__main__":
    unittest.main()
'''
