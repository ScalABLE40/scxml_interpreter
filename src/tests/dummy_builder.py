#!/usr/bin/env python

from ..skeleton_class import *
from ..smach_builder import SmachBuilder

def dummy_filler():
    ##SimpleState Filler
    WaitSetupSkel = SimpleStateSkeleton("WaitSkill_SETUP",{},{"preempted":"preempted","aborted":"aborted","succeeded":"WaitSkill_EXECUTION"})
    WaitExecuteSkel = SimpleStateSkeleton("WaitSkill_EXECUTION",{},{"preempted":"preempted","aborted":"aborted","succeeded":"WaitSkill_ANALYSIS"})
    WaitAnalysisSkel = SimpleStateSkeleton("WaitSkill_EXECUTION",{},{"preempted":"preempted","aborted":"aborted","succeeded":"succeeded"})
     
    ##CompoundFiller
    WaitSkillSkel = CoumpoundStateSkeleton("WaitSkill",{},{"preempted":"Final_4","aborted":"Final_4","succeeded":"Final_4"},
                                           [WaitSetupSkel,WaitExecuteSkel,WaitAnalysisSkel], "WaitSkill_SETUP")
    
    ##RootFiller
    RootStateSkel = RootStateSkeleton("WaitSkill",{},["Final_4"],[WaitSkillSkel])
    
    SCXMLSkel = SCXMLSkeleton()
    SCXMLSkel.simpleStates.append(WaitSetupSkel)
    SCXMLSkel.simpleStates.append(WaitExecuteSkel)
    SCXMLSkel.simpleStates.append(WaitAnalysisSkel)
    SCXMLSkel.coumpoundStates.append(WaitSkillSkel)
    SCXMLSkel.rootState.append(RootStateSkel)
    return SCXMLSkel
     
Skel = dummy_filler()
builder = SmachBuilder(Skel, provider)
SM = builder.build_state_machine()
SM.execute()
            