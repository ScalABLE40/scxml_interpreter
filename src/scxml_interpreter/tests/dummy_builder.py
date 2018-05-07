#!/usr/bin/env python

import rospy
from scxml_interpreter.skeleton_class import *
from scxml_interpreter.smach_builder import SmachBuilder
from scxml_interpreter.skill_provider import SkillProvider
from scxml_interpreter.WaitSkill import WaitSkill_SETUP

def dummy_filler():
    ##SimpleState Filler
    WaitSetupSkel = SimpleStateSkeleton("WaitSkill_SETUP",{},
                                        {"preempted":"preempted","aborted":"aborted","succeeded":"WaitSkill_EXECUTION"})
    WaitExecuteSkel = SimpleStateSkeleton("WaitSkill_EXECUTION",{},{"preempted":"preempted","aborted":"aborted","succeeded":"WaitSkill_ANALYSIS"})
    WaitAnalysisSkel = SimpleStateSkeleton("WaitSkill_ANALYSIS",{},{"preempted":"preempted","aborted":"aborted","succeeded":"succeeded"})
     
    ##CompoundFiller
    WaitSkillSkel = CoumpoundStateSkeleton("WaitSkill",{"actionName":"'/WaitSkill'","actionGoal":"{'waitTime':2.0}",
                                                   "actionType":"'wait_skill_msgs/WaitSkillAction'","actionResult":"{}"},{"preempted":"Final_4","aborted":"Final_4","succeeded":"Final_4"},
                                           [WaitSetupSkel,WaitExecuteSkel,WaitAnalysisSkel], "WaitSkill_SETUP")
    
    ##RootFiller
    RootStateSkel = RootStateSkeleton("WaitSkill",{},["Final_4"],[WaitSkillSkel])
    
    SCXMLSkel = SCXMLSkeleton()
    SCXMLSkel.simpleStates.append(WaitSetupSkel)
    SCXMLSkel.simpleStates.append(WaitExecuteSkel)
    SCXMLSkel.simpleStates.append(WaitAnalysisSkel)
    SCXMLSkel.coumpoundStates.append(WaitSkillSkel)
    SCXMLSkel.rootState = RootStateSkel
    return SCXMLSkel
     
     
     
rospy.init_node("test_scxml_interpreter")
Skel = dummy_filler()
Provider = SkillProvider("scxml_interpreter")
builder = SmachBuilder(Skel,Provider)
SM = builder.build_state_machine()
SM.execute()
            