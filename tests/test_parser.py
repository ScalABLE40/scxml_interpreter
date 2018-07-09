#!/usr/bin/env python
import rospy
import rospkg
import os
from scxml_interpreter.scxml_parser import SCXMLParser
from scxml_interpreter.interface_class import *
import unittest

class test_cases(unittest.TestCase):
    def test_rootstate(self):

        pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
        scxml_file = os.path.join(pkg_path, "resources/scxml/simple_file.scxml")
        scxml_parser = SCXMLParser()
        SCXMLSkeleton = scxml_parser.parsing_scxml(scxml_file)
        SCXMLRootSkeleton = scxml_parser.root_Interface(SCXMLSkeleton)
        self.assertEqual(str(SCXMLRootSkeleton), str(RootStateInterface('WaitSkill',{},["Final_4"],['WaitSkill'])))

    def test_compoundstate(self):

        pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
        scxml_file = os.path.join(pkg_path, "resources/scxml/simple_file.scxml")
        scxml_parser = SCXMLParser()
        SCXMLSkeleton = scxml_parser.parsing_scxml(scxml_file)
        compound_states=scxml_parser.get_compoundstates()
        for i in range(len(compound_states)):
            test_compoundstates=compound_states[i]
        self.assertEqual(str(test_compoundstates) ,str(CompoundStateInterface("WaitSkill",{"actionName":'"/WaitSkill"',"actionType":"'wait_skill_msgs/WaitSkillAction'","actionGoal":'{"waitTime":15.0}',"actionResult":'{}'},{'preempted':'Final_4','succeeded':'Final_4','aborted':'Final_4'},["WaitSkill_EXECUTION","WaitSkill_SETUP","WaitSkill_ANALYSIS"], "WaitSkill_SETUP")))


    def test_compoundstate_wronganalysis(self):

        pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
        scxml_file = os.path.join(pkg_path, "resources/scxml/simple_file.scxml")
        scxml_parser = SCXMLParser()
        SCXMLSkeleton = scxml_parser.parsing_scxml(scxml_file)
        compound_states=scxml_parser.get_compoundstates()
        for i in range(len(compound_states)):
            test_compoundstates=compound_states[i]
        self.assertNotEqual(str(test_compoundstates),str(CompoundStateInterface("WaitSkill",{"actionName":'"/waitskill"',"actionType":"wait_skill_msgs/WaitSkillAction","actionGoal":'{"waitTime":20.0}',"actionResult":'{}'},{"preempted":"Final_4","aborted":"Final_4","succeeded":"Final_4"},["WaitSkill_EXECUTION","WaitSkill_SETUP","WaitSkill_ANALYSIS"], "WaitSkill_SETUP")))

    def test_simplestate_analysis(self):
        result= {"WaitSkill_EXECUTION":SimpleStateInterface("WaitSkill_EXECUTION",{'stage': 'execution'},{'preempted': 'preempted','succeeded': 'WaitSkill_ANALYSIS','aborted': 'aborted'}),
        "WaitSkill_SETUP":SimpleStateInterface("WaitSkill_SETUP",{'stage': 'setup'},{'preempted': 'preempted','succeeded': 'WaitSkill_EXECUTION', 'aborted': 'aborted'}),
        "WaitSkill_ANALYSIS":SimpleStateInterface("WaitSkill_ANALYSIS",{'stage': 'analysis'},{'preempted':'preempted','aborted':'aborted','succeeded':'succeeded'})}
        test_simplestates=[]
        pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
        scxml_file = os.path.join(pkg_path, "resources/scxml/simple_file.scxml")
        scxml_parser = SCXMLParser()
        SCXMLSkeleton = scxml_parser.parsing_scxml(scxml_file)
        simple_states=scxml_parser.get_simplestates()
        for i in range(len(simple_states)):
            test_simplestates.append(simple_states[i])
        for id in result:
            for test_sstates in test_simplestates:
                if id ==test_sstates.id:
                    test_result=result[id]
                    self.assertEqual(str(test_sstates),str(test_result))


    def test_simplestate_wrong_analysis(self):
        result= {"WaitSkill_EXECUTION":SimpleStateInterface("WaitSkill_EXECUTION",{'stage': 'execution'},{'preempted': 'preempted','succeeded': 'WaitSkill_ANALYSIS','aborted': 'aborted'}),
        "WaitSkill_SETUP":SimpleStateInterface("WaitSkill_SETUP",{'stage': 'setup'},{'preempted': 'preempted','succeeded': 'WaitSkill_EXECUTION', 'aborted': 'aborted'}),
        "WaitSkill_ANALYSIS":SimpleStateInterface("WaitSkill_ANALYSIS",{'stage': 'analysis'},{'preempted':'preempted','aborted':'aborted','succeeded':'succeeded'})}
        test_simplestates=[]
        pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
        scxml_file = os.path.join(pkg_path, "resources/scxml/simple_file.scxml")
        scxml_parser = SCXMLParser()
        SCXMLSkeleton = scxml_parser.parsing_scxml(scxml_file)
        simple_states=scxml_parser.get_simplestates()
        for id in result:
            for i in range(len(simple_states)):
                test_simplestates.append(simple_states[i])
            for test_sstates in test_simplestates:
                if id !=test_sstates.id:
                    test_result=result[id]
                    self.assertNotEqual(str(test_sstates),str(result[id]))

'''
    def test_simplestate_execution(self):

        pkg_path = rospkg.RosPack().get_path("scxml_manager")
        scxml_file = os.path.join(pkg_path, "resources/simple_file.scxml")
        scxml_parser = SCXMLParser()
        SCXMLSkeleton = scxml_parser.parsing_scxml(scxml_file)
        simple_state=scxml_parser.simplestates
        for states in simple_state:
                skelteonsimple=scxml_parser.Interface_simplestate(test)
        self.assertEqual(str(skelteonsimple),str(SimpleStateInterface("WaitSkill_EXECUTION",{'stage': 'execution'},{"preempted":"preempted","succeeded":"WaitSkill_ANALYSIS","aborted":"aborted"})))

    def test_simplestate_analysis(self):

        pkg_path = rospkg.RosPack().get_path("scxml_manager")
        scxml_file = os.path.join(pkg_path, "resources/simple_file.scxml")
        scxml_parser = SCXMLParser()
        SCXMLSkeleton = scxml_parser.parsing_scxml(scxml_file)
        SCXMLsimple=scxml_parser.get_simplestates()
        for test_simple in SCXMLsimple:
            skelteoncomp=scxml_parser.Interface_simplestate(test_simple)
        self.assertEqual(str(skelteoncomp),str(SimpleStateSkeleton("WaitSkill_ANALYSIS",{'stage': 'analysis'},
        {"preempted":"preempted","aborted":"aborted","succeeded":"succeeded"})))

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
