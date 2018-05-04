#!/usr/bin/env python

import rospy
import smach
import importlib
import actionlib


class WaitSkill_SETUP(smach.State):
    def __init__(self, outcomes=["preempted","aborted","succeeded"], io_keys=["actionName","actionGoal","actionType","actionResult"]):
        smach.State.__init__(self, outcomes, io_keys)
        
    def load_type(self, type):
        pkg = type[:type.find('/',0,len(type))]
        module = type[type.find('/',0,len(type))+1:-6] 
        try:
            module_ref = importlib.import_module('.msg', pkg)
        except Exception as e:
            print(str(e))
        type_instance = module_ref.__getattribute__(module+"Action")()
        goal_instance = module_ref.__getattribute__(module+"Goal")()
        result_instance = module_ref.__getattribute__(module+"Result")()
        return type_instance, goal_instance, result_instance
        
        
    def execute(self, ud):
        type, goal, result = self.load_type(ud.actionType)
        for data, value in ud.actionGoal.iteritems():
            if(hasattr(goal, data)):
                setattr(goal, data, value)
            else:
                #raise warning ?
                pass
        ud.actionGoal = goal
        ud.actionType = type
        ud.actionResult = result
        return "succeeded"

class WaitSkill_EXECUTION(smach.State):
    def __init__(self, outcomes=["preempted","aborted","succeeded"], io_keys=["actionName","actionGoal","actionType","actionResult"]):
        smach.State.__init__(self, outcomes, io_keys)
        
        
    def execute(self, ud):
        client = actionlib.SimpleActionClient(ud.actionName,ud.actionType)
        client.wait_for_server()
        result = client.send_goal_and_wait(ud.actionGoal)
        ud.actionResult = result
        return "succeeded"

class WaitSkill_ANALYSIS(smach.State):
    def __init__(self, outcomes=["preempted","aborted","succeeded"], io_keys=["actionName","actionGoal","actionType","actionResult"]):
        smach.State.__init__(self, outcomes, io_keys)
        
        
    def execute(self, ud):
        return "succeeded"