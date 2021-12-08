#!/usr/bin/env python3

import rospy
import re
import xml.etree.ElementTree as etree
import os

from scxml_interpreter.interfaces import SimpleStateInterface, CompoundStateInterface, SCXMLInterface, ParallelStateInterface


def convert_to_concurence_map(cond):
    # split mapping
    cond_ = cond.strip()
    map_list = []
    if cond_.find(" AND ") == -1:  # There is no "and" condition
        if cond_.find(" OR ") == -1:  # There is no "or" condition
            list_ = cond_.split(".")
            map_list.append({list_[0]: list_[1]})
        else:
            for cond_or in cond_.split(" OR "):
                list_ = cond_or.split(".")
                map_list.append({list_[0]: list_[1]})
    else:
        for cond_and in cond_.split(" AND "):
            list_and = cond_and.split(".")
            map_list.append({list_and[0]: list_and[1]})
    return map_list


class Converter(object):
    def convert_to_interface(self, root):
        initial = root.attrib.get("initial")
        if initial is None:
            raise KeyError("No inital state found! ")
        final_states = self.convert_final_states(root, "Root SM")
        data = self.convert_datamodel(root, "RootSM")
        states = self.get_child_interfaces(root)
        interface = SCXMLInterface(data, final_states, states, initial)
        return interface

    def convert_datamodel(self, xml_node, state_id):
        datamodel_dict = {}
        for data in xml_node.findall("./datamodel/data"):
            data_id = data.attrib.get("id")
            data_expr = data.attrib.get("expr")
            if data_id is None:
                raise KeyError("No data_id in datamodel for state: %s" % state_id)
            if data_expr is None:
                raise KeyError("No data_expr in datamodel for state: %s" % state_id)
            datamodel_dict[data_id] = data_expr
        return datamodel_dict

    def convert_transitions(self, xml_node, state_id):
        transitions = {}
        for transition in xml_node.findall("transition"):
            event = transition.attrib.get("event")
            target = transition.attrib.get("target")
            if event is None:
                raise KeyError("No event in transition for state: %s" % state_id)
            if target is None:
                raise KeyError("No target in transition for state: %s" % state_id)
            transitions[event] = target
        return transitions

    def convert_parallel_transitions(self, xml_node, state_id):
        transitions = {}
        default = "failed"
        outcome_map = {}
        for transition in xml_node.findall("transition"):
            event = transition.attrib.get("event")
            target = transition.attrib.get("target")
            cond = transition.attrib.get("cond")
            if event is None:
                raise KeyError("No Transition Event for %s !" % state_id)
            if target is None:
                raise KeyError("No Transition Target for %s !" % state_id)
            if cond is None:
                if event != default and event not in transitions.keys():
                    raise KeyError("No Transition Condition for %s !" % state_id)
            else:
                map_list = convert_to_concurence_map(cond)
                outcome_map[event] = map_list
                transitions[event] = target
            if event == default and (default not in transitions.keys()):
                transitions[default] = default
        if default is None:
            rospy.logwarn("No default outcome defined for Parallel %s !" % state_id)
        return transitions, default, outcome_map

    def convert_final_states(self, xml_node, state_id):
        final_states_id = []
        final_states = xml_node.findall("./final")
        if len(final_states) == 0:
            raise KeyError("No final state for coumpound state: %s" % state_id)
        for state in final_states:
            final_id = state.attrib.get("id")
            if final_id is None:
                raise KeyError("No id for the final state in coumpound state: %s" % state_id)
            final_states_id.append(final_id)
        return final_states_id

    def get_child_interfaces(self, xml_node):
        states = []
        for node in xml_node.findall("./state"):
            states.append(self.create_child_interface(node))
        for node in xml_node.findall("./parallel"):
            states.append(self.create_parallelstate_interface(node))
        return states

    def create_child_interface(self, xml_node):
        if len(xml_node.findall("./state")) == 0 and len(xml_node.findall("./parallel")) == 0:
            return self.create_simplestate_interface(xml_node)
        else:
            return self.create_compoundstate_interface(xml_node)

    def create_simplestate_interface(self, node):
        node_id = node.attrib.get("id")
        if node_id is None:
            raise KeyError("[SCXML Interpreter] No ID found for a simple state!")
        transition = self.convert_transitions(node, node_id)
        datamodel = self.convert_datamodel(node, node_id)
        interface = SimpleStateInterface(node_id, datamodel, transition)
        return interface

    def create_compoundstate_interface(self, node):
        node_id = node.attrib.get("id")
        if node_id is None:
            raise KeyError("[SCXML Interpreter] No ID found for a compound state!")
        initial = node.find("./initial/transition").attrib.get("target")
        transition = self.convert_transitions(node, node_id)
        if not (transition):
            for final_state in self.convert_final_states(node, node_id):
                transition[final_state] = final_state
        datamodel = self.convert_datamodel(node, node_id)
        states = self.get_child_interfaces(node)
        return CompoundStateInterface(node_id, datamodel, transition, states, initial)

    def create_parallelstate_interface(self, node):
        node_id = node.attrib.get("id")
        if node_id is None:
            raise KeyError("[SCXML Interpreter] No ID found for a simple state!")
        transitions, default, outcome_map = self.convert_parallel_transitions(node, node_id)
        datamodel = self.convert_datamodel(node, node_id)
        states = self.get_child_interfaces(node)
        return ParallelStateInterface(node_id, datamodel, default, outcome_map, transitions, states)


class SCXMLParser(Converter):
    """Parsing all the details for constructing the state machine from scxml machine"""

    def parsing_scxml(self, scxml_file):
        try:
            ParseError = etree.ParseError
        except ImportError:
            from xml.parsers import expat

            ParseError = expat.ExpatError
        try:

            if os.path.splitext(scxml_file)[1] == ".scxml":
                file_ = open(scxml_file).read()
                file_ = re.sub(' xmlns="[^"]+"', "", file_, count=1)
        except ParseError as ex:
            rospy.logerr(ex)
        return self.parsing_string(file_)

    def parsing_string(self, string):
        try:
            root = etree.fromstring(string)
        except Exception as ex:
            rospy.logerr(ex)
            rospy.logerr("Parsing is not correct or the file is not SCXML")
            raise ex
        return self.convert_to_interface(root)
