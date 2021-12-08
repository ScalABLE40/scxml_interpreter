#!/usr/bin/env python3

import rospy
import smach


class dummy_print(smach.State):
    def __init__(self, outcomes=["succeeded"], io_keys=["print_text"]):
        smach.State.__init__(self, outcomes, io_keys=io_keys)

    def execute(self, ud):
        print(ud._ud)
        if ud.print_text == "":
            text_ = "Hello World ! I'm a dummy printing state"
        else:
            text_ = ud.print_text
        rospy.loginfo(text_)
        rospy.sleep(1.0)
        return "succeeded"


class dummy_outcome(smach.State):
    def __init__(self, outcomes=["out0", "out1", "out2"], io_keys=["outcome"]):
        smach.State.__init__(self, outcomes, io_keys=io_keys)

    def execute(self, ud):
        rospy.sleep(1.0)
        if not (ud.outcome):
            rospy.loginfo("My outcome is not defined. I will move to out0 !")
            return "out0"
        if ud.outcome == "out1":
            rospy.loginfo("My outcome is defined to %s" % (str(ud.outcome)))
            return "out1"
        elif ud.outcome == "out2":
            rospy.loginfo("My outcome is defined to %s" % (str(ud.outcome)))
            return "out2"
        else:
            rospy.loginfo("Unknown outcom %s. Will return out0 !" % (str(ud.outcome)))
            return "out0"
        return "succeeded"
