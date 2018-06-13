#!/usr/bin/env python

import rospy
import rospkg
import os
from scxml_interpreter.scxml_parser import SCXMLParser
from scxml_interpreter.skeleton_class import RootStateSkeleton,CompoundStateSkeleton,SimpleStateSkeleton,ParallelStateSkeleton
import unittest
import xml.etree.ElementTree as etree


class test_cases(unittest.TestCase):
    def test_rootstate(self):

        pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
        scxml_file = os.path.join(pkg_path, "resources/wait_skill.scxml")
        scxml_parser = SCXMLParser()
        SCXMLSkeleton = scxml_parser.parsing_scxml(scxml_file)
        SCXMLRootSkeleton = scxml_parser.root_skeleton(SCXMLSkeleton)
        self.assertEqual(str(SCXMLRootSkeleton), str(RootStateSkeleton('WaitSkill',{},["Final_4"],['WaitSkill'])))

    def test_compoundstate(self):

        pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
        scxml_file = os.path.join(pkg_path, "resources/wait_skill.scxml")
        scxml_parser = SCXMLParser()
        SCXMLSkeleton = scxml_parser.parsing_scxml(scxml_file)
        SCXMlgetcompoundstate=scxml_parser.get_compoundstates()
        for test_compound in SCXMlgetcompoundstate:
            SCXMLcompound=scxml_parser.skeleton_compoundstate(test_compound)
        self.assertEqual(str(SCXMLcompound),str(CompoundStateSkeleton("WaitSkill",{"actionName":'"/waitskill"',"actionType":"waitskill/waitskill","actionGoal":'{"waitTime":2.0}',"actionResult":'{}'},{"preempted":"Final_4","aborted":"Final_4","succeeded":"Final_4"},["WaitSkill_SETUP","WaitSkill_EXECUTION","WaitSkill_ANALYSIS"], "WaitSkill_SETUP")))


    def test_compoundstate1(self):

        pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
        scxml_file = os.path.join(pkg_path, "resources/wait_skill.scxml")
        scxml_parser = SCXMLParser()
        SCXMLSkeleton = scxml_parser.parsing_scxml(scxml_file)
        SCXMlgetcompoundstate=scxml_parser.get_compoundstates()
        for test_compound in SCXMlgetcompoundstate:
            skelteoncomp=scxml_parser.skeleton_compoundstate(test_compound)
        self.assertEqual(str(skelteoncomp),str(CompoundStateSkeleton("WaitSkill",{"actionName":'"/waitskill"',"actionType":"waitskill/waitskill","actionGoal":'{"waitTime":2.0}',"actionResult":'{}'},{"preempted":"Final_4","aborted":"Final_4","succeeded":"Final_4"},["WaitSkill_SETUP","WaitSkill_EXECUTION","WaitSkill_ANALYSIS"], "WaitSkill_SETUP")))

    def test_simplestate_setup(self):

        pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
        scxml_file = os.path.join(pkg_path, "resources/wait_skill_setup.scxml")
        scxml_parser = SCXMLParser()
        SCXMLSkeleton = scxml_parser.parsing_scxml(scxml_file)
        SCXMLsimple=scxml_parser.get_simplestates()
        for test_simple in SCXMLsimple:
            skelteoncomp=scxml_parser.skeleton_simplestate(test_simple)
        self.assertEqual(str(skelteoncomp),str(SimpleStateSkeleton("WaitSkill_SETUP",{'stage': 'setup'},
        {"preempted":"preempted","succeeded":"WaitSkill_EXECUTION","aborted":"aborted"})))

    def test_simplestate_execution(self):

        pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
        scxml_file = os.path.join(pkg_path, "resources/wait_skill_execution.scxml")
        scxml_parser = SCXMLParser()
        SCXMLSkeleton = scxml_parser.parsing_scxml(scxml_file)
        SCXMLsimple=scxml_parser.get_simplestates()
        for test_simple in SCXMLsimple:
            skelteoncomp=scxml_parser.skeleton_simplestate(test_simple)
        self.assertEqual(str(skelteoncomp),str(SimpleStateSkeleton("WaitSkill_EXECUTION",{'stage': 'execution'},
        {"preempted":"preempted","succeeded":"WaitSkill_ANALYSIS","aborted":"aborted"})))

    def test_simplestate_analysis(self):

        pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
        scxml_file = os.path.join(pkg_path, "resources/wait_skill_analysis.scxml")
        scxml_parser = SCXMLParser()
        SCXMLSkeleton = scxml_parser.parsing_scxml(scxml_file)
        SCXMLsimple=scxml_parser.get_simplestates()
        for test_simple in SCXMLsimple:
            skelteoncomp=scxml_parser.skeleton_simplestate(test_simple)
        self.assertEqual(str(skelteoncomp),str(SimpleStateSkeleton("WaitSkill_ANALYSIS",{'stage': 'analysis'},
        {"preempted":"preempted","aborted":"aborted","succeeded":"succeeded"})))

    def test_simplestate_wrong_analysis(self):

        pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
        scxml_file = os.path.join(pkg_path, "resources/wait_skill_analysis.scxml")
        scxml_parser = SCXMLParser()
        SCXMLSkeleton = scxml_parser.parsing_scxml(scxml_file)
        SCXMLsimple=scxml_parser.get_simplestates()
        for test_simple in SCXMLsimple:
            skelteoncomp=scxml_parser.skeleton_simplestate(test_simple)
        self.assertNotEqual(str(skelteoncomp),str(SimpleStateSkeleton("WaitSkill_EXECUTION",{'stage': 'analysis'},
        {"preempted":"preempted","aborted":"aborted","succeeded":"succeeded"})))

    '''
    def test_parallelstate(self):
        pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
        scxml_file = os.path.join(pkg_path, "resources/scxml/parallel.scxml")
        scxml_parser = SCXMLParser()
        SCXMLSkeleton = scxml_parser.parsing_scxml(scxml_file)
        SCXMLParallel = scxml_parser.get_parallelstates()
        for test_parallel in SCXMLParallel:
            skeletonparallel = scxml_parser.skeleton_parallelstate(test_parallel)
        self.assertEqual(str(skeletonparallel),str(ParallelStateSkeleton("Parallel_2",{'test': '14054067'},{'failed': 'State_7', 'success': 'State_5'},
        ['State_3','State_4','State_4_Bis'],None,None,({'test': 'Test value :', 'outcome': 'test'}, None))))
    '''
if __name__ == "__main__":
    unittest.main()
