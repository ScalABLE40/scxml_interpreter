#!/usr/bin/env python

import rospy
import rospkg
import os
from scxml_interpreter.scxml_parser import SCXMLParser
from scxml_interpreter.skeleton_class import RootStateSkeleton,CompoundStateSkeleton,SimpleStateSkeleton
import unittest
import xml.etree.ElementTree as etree


class Testcases(unittest.TestCase):
       def test_rootstate(self):

           pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
           scxml_file = os.path.join(pkg_path, "resources/scxml/wait_skill.scxml")
           scxml_parser = SCXMLParser()
           SCXMLSkeleton = scxml_parser.parcing_scxml(scxml_file)
           SCXMLRootSkeleton = scxml_parser.root_skeleton(SCXMLSkeleton)
           self.assertEqual(str(SCXMLRootSkeleton), str(RootStateSkeleton('WaitSkill',{},["Final_4"],['WaitSkill'])))

       def test_compoundstate(self):

           pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
           scxml_file = os.path.join(pkg_path, "resources/scxml/wait_skill.scxml")
           scxml_parser = SCXMLParser()
           SCXMLSkeleton = scxml_parser.parcing_scxml(scxml_file)
           SCXMLcompound=scxml_parser.skeleton_all_compoundstates()
           skelteoncomp=scxml_parser.skeleton_compoundstate(SCXMLcompound)
           self.assertEqual(str(skelteoncomp),str(CompoundStateSkeleton("WaitSkill",{"actionName":'"/waitskill"',"actionType":"waitskill/waitskill","actionGoal":'{"waitTime":2.0}',"actionResult":'{}'},{"preempted":"Final_4","aborted":"Final_4","succeeded":"Final_4"},["WaitSkill_SETUP","WaitSkill_EXECUTION","WaitSkill_ANALYSIS"], "WaitSkill_SETUP")))

       def test_compoundstate1(self):

           pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
           scxml_file = os.path.join(pkg_path, "resources/scxml/wait_skill.scxml")
           scxml_parser = SCXMLParser()
           SCXMLSkeleton = scxml_parser.parcing_scxml(scxml_file)
           SCXMLcompound=scxml_parser.skeleton_all_compoundstates()
           skelteoncomp=scxml_parser.skeleton_compoundstate(SCXMLcompound)
           self.assertEqual(str(skelteoncomp),str(CompoundStateSkeleton("WaitSkill_",{"actionName":'"/waitskill"',"actionType":"waitskill/waitskill","actionGoal":'{"waitTime":2.0}',"actionResult":'{}'},{"preempted":"Final_4","aborted":"Final_4","succeeded":"Final_4"},["WaitSkill_SETUP","WaitSkill_EXECUTION","WaitSkill_ANALYSIS"], "WaitSkill_SETUP")))

       def test_simplestate(self):

           pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
           scxml_file = os.path.join(pkg_path, "resources/scxml/wait_skill.scxml")
           scxml_parser = SCXMLParser()
           SCXMLSkeleton = scxml_parser.parcing_scxml(scxml_file)
           SCXMLsimple=scxml_parser.skeleton_all_simplestates()
           skelteoncomp=scxml_parser.skeleton_simplestate(SCXMLsimple)
           self.assertEqual(str(skelteoncomp),str(SimpleStateSkeleton("WaitSkill_SETUP",{'stage': 'setup'},
           {"preempted":"preempted","succeeded":"WaitSkill_EXECUTION","aborted":"aborted"})))


if __name__ == "__main__":
       unittest.main()


