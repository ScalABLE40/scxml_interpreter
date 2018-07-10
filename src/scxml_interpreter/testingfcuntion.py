from scxml_interpreter.Errorexecptions import *
import traceback
import rospy
import rospkg
import os
from scxml_interpreter.scxml_parser import SCXMLParser
from scxml_interpreter.interface_class import *
import unittest

def test_simplestate_analysis():
        result= {"WaitSkill_EXECUTION":SimpleStateInterface("WaitSkill_EXECUTION",{'stage': 'execution'},{'preempted': 'preempted','succeeded': 'WaitSkill_ANALYSIS','aborted': 'aborted'}),
        "WaitSkill_SETUP":SimpleStateInterface("WaitSkill_SETUP",{'stage': 'setup'},{'preempted': 'preempted','succeeded': 'WaitSkill_EXECUTION', 'aborted': 'aborted'}),
        "WaitSkill_ANALYSIS":SimpleStateInterface("WaitSkill_ANALYIS",{'stage': 'analysis'},{'preempted':'preempted','aborted':'aborted','succeeded':'succeeded'})}
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
                print ("test ste id %s"%test_sstates.id)
                print("ID %s"%id)
                if id ==test_sstates.id:
                    test_result=result[id]
                    print ("test_sstates %s"%test_sstates)
                    print("test_result %s"%result[id])


                    #self.assertEqual(str(test_sstates),str(test_result))

if __name__=="__main__":
    test=test_simplestate_analysis()
    print test
