import rospy
import smach

class StateMachine(smach.StateMachine):
    def __init__(self, outcomes, input_keys=[], output_keys=[]):
        smach.StateMachine.__init__(self, outcomes, input_keys, output_keys)
        self.register_outcomes(["preempt"])
        self._datamodel = {}

class SkillStateMachine(smach.StateMachine):
    