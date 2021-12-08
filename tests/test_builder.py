#!/usr/bin/env python3
import rospkg
import os
import smach

from scxml_interpreter.scxml_parser import SCXMLParser
from scxml_interpreter.interfaces import *
from scxml_interpreter.state_provider import SmachStateProvider
from scxml_interpreter.smach_builder import SmachBuilder

import unittest


class test_cases(unittest.TestCase):
    # testing the state of root of state machine
    STATE_ID = ["dummy_print1", "dummy_print_2", "State_6", "Parent", "dummy_print_child", "dummy_outcome"]

    def test_parser(self):
        pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
        scxml_file = os.path.join(pkg_path, "tests/resources/dummy_test.scxml")
        scxml_parser = SCXMLParser()
        scxml_parsing = scxml_parser.parsing_scxml(scxml_file)
        self.assertTrue(isinstance(scxml_parsing, SCXMLInterface))
        self.assertEqual(len(scxml_parsing.childs_interface), 4)

        def check_transition(state, transition_name, transition_target):
            self.assertIn(transition_name, state.transitions.keys())
            self.assertIn(transition_target, state.transitions.values())

        for child in scxml_parsing.childs_interface:
            self.assertTrue(isinstance(child.identifier, str))
            self.assertIn(child.identifier, self.STATE_ID)
            if child.identifier == "dummy_print1":
                check_transition(child, "succeeded", "dummy_print_2")
            if child.identifier == "dummy_print_2":
                check_transition(child, "succeeded", "State_6")
            if child.identifier == "State_6":
                check_transition(child, "succeeded", "dummy_outcome")
            if child.identifier == "dummy_outcome":
                check_transition(child, "out0", "out0")
                check_transition(child, "out1", "out1")
                check_transition(child, "out2", "out2")

    def test_builder(self):
        pkg_path = rospkg.RosPack().get_path("scxml_interpreter")
        scxml_file = os.path.join(pkg_path, "tests/resources/dummy_test.scxml")
        scxml_parser = SCXMLParser()
        scxml_parsing = scxml_parser.parsing_scxml(scxml_file)
        provider = SmachStateProvider("scxml_interpreter")
        builder = SmachBuilder(scxml_parsing, [provider])
        SM, ud = builder.build_state_machine()
        self.assertIsInstance(SM, smach.StateMachine)
        result = SM.execute(ud)
        self.assertEqual(result, "out1")


if __name__ == "__main__":
    import rosunit

    rosunit.unitrun("scxml_interpreter", "test_cases", test_cases)
