#!/usr/bin/env python

import rospy
from scxml_interpreter.interface_class import *
from scxml_interpreter.smach_builder import SmachBuilder
from scxml_interpreter.state_provider import SmachStateProvider

def dummy_filler():
    ##SimpleState Filler
    WaitSetupSkel = SimpleStateInterface("WaitSkill_SETUP",{},
                                        {"preempted":"preempted","aborted":"aborted","succeeded":"WaitSkill_EXECUTION"})
    WaitExecuteSkel = SimpleStateInterface("WaitSkill_EXECUTION",{},{"preempted":"preempted","aborted":"aborted","succeeded":"WaitSkill_ANALYSIS"})
    WaitAnalysisSkel = SimpleStateInterface("WaitSkill_ANALYSIS",{},{"preempted":"preempted","aborted":"aborted","succeeded":"succeeded"})
     
    ##CompoundFiller
    WaitSkillSkel = CompoundStateInterface("WaitSkill",{"actionName":"'/WaitSkill'","actionGoal":"{'waitTime':2.0}",
                                                   "actionType":"'wait_skill_msgs/WaitSkillAction'","actionResult":"{}"},{"preempted":"Final_4","aborted":"Final_4","succeeded":"Final_4"},
                                           [WaitSetupSkel,WaitExecuteSkel,WaitAnalysisSkel], "WaitSkill_SETUP")
    
    ##RootFiller
    RootStateSkel = RootStateSkeleton("WaitSkill",{},["Final_4"],[WaitSkillSkel])
    
    SCXMLSkel = SCXMLInterface()
    SCXMLSkel.simpleStates.append(WaitSetupSkel)
    SCXMLSkel.simpleStates.append(WaitExecuteSkel)
    SCXMLSkel.simpleStates.append(WaitAnalysisSkel)
    SCXMLSkel.compoundStates.append(WaitSkillSkel)
    SCXMLSkel.rootState = RootStateSkel
    return SCXMLSkel
     
     
     
rospy.init_node("test_scxml_interpreter")
Skel = dummy_filler()
Provider = SmachStateProvider("scxml_interpreter")
builder = SmachBuilder(Skel,[Provider])
SM = builder.build_state_machine()
rospy.sleep(2.0)
SM.execute()
rospy.sleep(20.0)


            