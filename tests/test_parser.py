#!/usr/bin/env python
import rospy
import rospkg
import os
from scxml_interpreter.scxml_parser import SCXMLParser
from scxml_interpreter.interface_class import *
import unittest

class test_cases(unittest.TestCase):


    #testing the state of root of state machine
    def test_rootstate(self):

        pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
        scxml_file = os.path.join(pkg_path, "resources/scxml/simple_file.scxml")
        scxml_parser = SCXMLParser()
        SCXMLInterface = scxml_parser.parsing_scxml(scxml_file)
        SCXMLRootInterface = scxml_parser.root_Interface(SCXMLInterface)
        self.assertEqual(str(SCXMLRootInterface), str(RootStateInterface('WaitSkill',{},["Final_4"],['WaitSkill'])))
    #testing the compound states and it is equal to the expected ouput
    def test_compoundstate(self):
        result={"WaitSkill":CompoundStateInterface("WaitSkill",{"actionName":'"/WaitSkill"',"actionType":"'wait_skill_msgs/WaitSkillAction'","actionGoal":'{"waitTime":15.0}',"actionResult":'{}'},{'preempted':'Final_4','succeeded':'Final_4','aborted':'Final_4'},["WaitSkill_EXECUTION","WaitSkill_SETUP","WaitSkill_ANALYSIS"], "WaitSkill_SETUP")}
        test_compoundstates=[]
        pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
        scxml_file = os.path.join(pkg_path, "resources/scxml/simple_file.scxml")
        scxml_parser = SCXMLParser()
        SCXMLInterface = scxml_parser.parsing_scxml(scxml_file)
        compound_states=scxml_parser.get_compoundstates()
        for i in range(len(compound_states)):
            test_compoundstates.append(compound_states[i])
        for id in result:
            for test_cstates in test_compoundstates:
                if id ==test_cstates.id:
                    test_result=result[id]
                    self.assertEqual(str(test_cstates),str(test_result))

    #testing the compound states and it is not equal to the expected ouput
    def test_compoundstate_wrong_skill(self):
        result={"WaitSkill":CompoundStateInterface("Waitkill",{"actionName":'"/waitskill"',"actionType":"wait_skill_msgs/WaitSkillAction","actionGoal":'{"waitTime":20.0}',"actionResult":'{}'},{"preempted":"Final_4","aborted":"Final_4","succeeded":"Final_4"},["WaitSkill_EXECUTION","WaitSkill_SETUP","WaitSkill_ANALYSIS"], "WaitSkill_SETUP")}
        pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
        scxml_file = os.path.join(pkg_path, "resources/scxml/simple_file.scxml")
        scxml_parser = SCXMLParser()
        SCXMLInterface = scxml_parser.parsing_scxml(scxml_file)
        compound_states=scxml_parser.get_compoundstates()
        test_compoundstates=[]
        for i in range(len(compound_states)):
            test_compoundstates=compound_states[i]
        self.assertNotEqual(str(test_compoundstates),str(CompoundStateInterface("WaitSkill",{"actionName":'"/waitskill"',"actionType":"wait_skill_msgs/WaitSkillAction","actionGoal":'{"waitTime":20.0}',"actionResult":'{}'},{"preempted":"Final_4","aborted":"Final_4","succeeded":"Final_4"},["WaitSkill_EXECUTION","WaitSkill_SETUP","WaitSkill_ANALYSIS"], "WaitSkill_SETUP")))

    #testing the simple states and it is equal to the expected ouput
    def test_simplestate_together(self):
        result= {"WaitSkill_EXECUTION":SimpleStateInterface("WaitSkill_EXECUTION",{'stage': 'execution'},{'preempted': 'preempted','succeeded': 'WaitSkill_ANALYSIS','aborted': 'aborted'}),
        "WaitSkill_SETUP":SimpleStateInterface("WaitSkill_SETUP",{'stage': 'setup'},{'preempted': 'preempted','succeeded': 'WaitSkill_EXECUTION', 'aborted': 'aborted'}),
        "WaitSkill_ANALYSIS":SimpleStateInterface("WaitSkill_ANALYSIS",{'stage': 'analysis'},{'preempted':'preempted','aborted':'aborted','succeeded':'succeeded'})}
        test_simplestates=[]
        pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
        scxml_file = os.path.join(pkg_path, "resources/scxml/simple_file.scxml")
        scxml_parser = SCXMLParser()
        SCXMLInterface = scxml_parser.parsing_scxml(scxml_file)
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


    def test_simplestate_execution(self):

        pkg_path = rospkg.RosPack().get_path("scxml_manager")
        scxml_file = os.path.join(pkg_path, "resources/simple_file.scxml")
    #testing the simple states and it is not equal to the expected ouput
    def test_simplestate_wrong_analysis(self):
        result= {"WaitSkill_ANALYSIS":SimpleStateInterface("WaitSkill_ANLYSIS",{'stage': 'analysis'},{'preempted':'preempted','aborted':'aborted','succeeded':'succeeded'})}
        test_simplestates=[]
        pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
        scxml_file = os.path.join(pkg_path, "resources/scxml/simple_file.scxml")
        scxml_parser = SCXMLParser()
        SCXMLInterface = scxml_parser.parsing_scxml(scxml_file)
        simple_states=scxml_parser.get_simplestates()
        for i in range(len(simple_states)):
            test_simplestates.append(simple_states[i])
        for id in result:
            for test_sstates in test_simplestates:
                if id ==test_sstates.id:
                    test_result=result[id]
                    self.assertNotEqual(str(test_sstates),str(result[id]))
   #testing the simple states and it is not equal to the expected ouput
    def test_simplestate_wrong_execution(self):
        result= {"WaitSkill_EXECUTION":SimpleStateInterface("WaitSkill_EXECUTION",{'stage': 'execuion'},{'preempted': 'preempted','succeeded': 'WaitSkill_ANALYSIS','aborted': 'aborted'})}
        #"WaitSkill_SETUP":SimpleStateInterface("WaitSkill_SETUP",{'stage': 'setup'},{'preempted': 'preemptd','succeeded': 'WaitSkill_EXECUTION', 'aborted': 'aborted'}),
        #"WaitSkill_ANALYSIS":SimpleStateInterface("WaitSkill_ANLYSIS",{'stage': 'analysis'},{'preempted':'preempted','aborted':'aborted','succeeded':'succeeded'})}
        test_simplestates=[]
        pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
        scxml_file = os.path.join(pkg_path, "resources/scxml/simple_file.scxml")
        scxml_parser = SCXMLParser()
        SCXMLInterface = scxml_parser.parsing_scxml(scxml_file)
        simple_states=scxml_parser.get_simplestates()
        for i in range(len(simple_states)):
            test_simplestates.append(simple_states[i])
        for id in result:
            for test_sstates in test_simplestates:
                if id ==test_sstates.id:
                    test_result=result[id]
                    self.assertNotEqual(str(test_sstates),str(result[id]))
#testing the simple states and it is not equal to the expected ouput
    def test_simplestate_wrong_setup(self):
        result= {"WaitSkill_SETUP":SimpleStateInterface("WaitSkill_SETUP",{'stage': 'setup'},{'preempted': 'preemptd','succeeded': 'WaitSkill_EXECUTION', 'aborted': 'aborted'})}
        test_simplestates=[]
        pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
        scxml_file = os.path.join(pkg_path, "resources/scxml/simple_file.scxml")
        scxml_parser = SCXMLParser()
        SCXMLInterface = scxml_parser.parsing_scxml(scxml_file)
        simple_states=scxml_parser.get_simplestates()
        for i in range(len(simple_states)):
            test_simplestates.append(simple_states[i])
        for id in result:
            for test_sstates in test_simplestates:
                if id ==test_sstates.id:
                    test_result=result[id]
                    self.assertNotEqual(str(test_sstates),str(result[id]))

'''
    def test_parallelstate(self):
        pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
        scxml_file = os.path.join(pkg_path, "resources/scxml/parallel.scxml")
        scxml_parser = SCXMLParser()
        SCXMLInterface = scxml_parser.parsing_scxml(scxml_file)
        SCXMLParallel = scxml_parser.get_parallelstates()
        for test_parallel in SCXMLParallel:
            skeletonparallel = scxml_parser.skeleton_parallelstate(test_parallel)
        self.assertEqual(str(skeletonparallel),str(ParallelStateSkeleton("Parallel_2",{'test': '14054067'},{'failed': 'State_7', 'success': 'State_5'},
        ['State_3','State_4','State_4_Bis'],None,None,({'test': 'Test value :', 'outcome': 'test'}, None))))
'''

if __name__ == "__main__":
    unittest.main()
