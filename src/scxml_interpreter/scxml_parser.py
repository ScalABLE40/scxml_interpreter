#!/usr/bin/env python

import rospy
import re
from roslib.packages import get_pkg_dir
import xml.etree.ElementTree as etree
from skill_provider import SkillProvider
#from skill_class import StateMachine
import smach
import sys
from os import path
import os
from scxml_interpreter.skeleton_class import *

class SCXMLParser:
    def __init__(self):
        self.isfile=True
        self.simplestates_=[]
        self.compoundstates_=[]
        self.finalstates_=[]
        preprocess_mapping = {}
##pasring the file correctly####
    def parcing_scxml(self,scxml_file):
        try:
            ParseError = etree.ParseError
        except ImportError:
            from xml.parsers import expat
            ParseError = expat.ExpatError
        try:
##file open  and file check if the file is scxml or not####
            if os.path.splitext(scxml_file)[1] == ".scxml":
                file_ = open(scxml_file).read()
                file_ = re.sub(' xmlns="[^"]+"', '', file_, count=1)              
                self.root = etree.fromstring(file_)
                self.root_skeleton(self.root)
        except ParseError as ex:
            rospy.logerr(ex)
            rospy.logerr('parsing is not correct is not SCXML')

        return self.root
#########creating all the compound states in the scxml######
    def skeleton_all_compoundstates(self):
        states_=[]
        for node in self.compoundstates_:
            if(node is not None):
                states_.append(self.skeleton_compoundstate(node))
        return node
######Each compound state provides all info state,transitions######
    def skeleton_compoundstate(self,node):
        states=[]
        node_id_compund=node.attrib.get('id')
        initial=node.find('./initial/transition').attrib.get('target')
        if(node_id_compund is not None):
            transition=self.get_transition(node)
            datamodel=self.get_datamodel(node)
        else:
            rospy.logerr("[SCXML Interpreter] No ID found!!!!")
        for state in node.findall('./state'):
            test=state.attrib.get('id')
            states.append(test)
        skeleton = CompoundStateSkeleton(node_id_compund,datamodel, transition,states,initial)
        return skeleton

#########creating all the simple states in the scxml######
    def skeleton_all_simplestates(self):
      states=[]
      for node in self.simplestates_:
            if(node is not None):
                states.append(self.skeleton_simplestate(node))
      return node
######Each simple state provides all info state,transitions######
    def skeleton_simplestate(self,node):
        node_id=node.attrib.get('id')
        if(node_id is not None):
            transition=self.get_transition(node)
            datamodel=self.get_datamodel(node)
        skeleton = SimpleStateSkeleton(node_id,datamodel,transition)
        return skeleton

#######transitions details for all states###############
    def get_transition(self,current_state):
        current=current_state.attrib.get('id')
        transitions_={}
        target=[]
        if(current is not None):
            for transition in current_state.findall("transition"):
                event = transition.attrib.get('event')
                if(event is None):
                    rospy.logerr("[SCXML Interpreter] No Transition Event for %s !!!!")
                    return
                target_state = transition.attrib.get('target')
                if(target_state is None):
                    rospy.logerr("[SCXML Interpreter] No Transition Target for %s !!!!" )
                    return
                target.append(target_state)
                transitions_[event] = target_state
        return transitions_

########datamodels and data for all state#############
    def get_datamodel(self,current_data):
        datamodel_ = {}
        for data in current_data.findall('./datamodel/data'):
            if( data is not None):
                data_ID = data.attrib['id']
                if('expr' in data.attrib):
                    data_expr = data.attrib['expr']
                else:
                    rospy.logwarn('No datamodel presnt')
                datamodel_[data_ID] = data_expr
            else:
                rospy.logerr("[SCXML Interpreter] Data is empty")
        return datamodel_
########final state#############
    def get_final_states_id(self):
        final_states = self.root.findall('./final')
        final_states_id = []
        if(len(final_states) == 0):
            rospy.logerr("[SCXML Interpreter] No final State found for the state")
            return None
        for state in final_states:
            final_id = state.attrib.get('id')
            if(final_id == None):
                rospy.logerr("[SCXML Interpreter] No id for the final state ")
            else:
                final_states_id.append(final_id)
        return final_states_id

    def root_skeleton(self,root):
        root=self.root
        self.current_states=[]
        self.allstates=[]
        states=[]
        initial=root.attrib.get('initial')
        if(initial is not None):
            for state in root.findall('.//state'):
                for init in state.findall('./initial/transition'):
                    init_target=init.attrib.get('target')
                    event = init.attrib.get('event')
###check the inital state and the check the state is compound or simple state####
            for node in self.root.findall('.//state'):
                ID=node.attrib.get('id')
                for transition in node.findall('.//transition'):
                    target_state=transition.attrib.get('target')
                    self.allstates.append(target_state)
            for node in self.root.findall('state'):
                ID=node.attrib.get('id')
                states.append(ID)
            self.get_compundstates()
            self.get_simplestates()

        else:
            rospy.logerr("[SCXML Interpreter] could not find initial state")
        final_states=self.get_final_states_id()
        data=self.get_datamodel(root)
        skeleton=RootStateSkeleton(initial,data,final_states,states)
        return skeleton

 ####get states#######
    def get_compundstates(self):
        compoundstates=[]
        for node in self.root.findall('.//state'):
            node_state=node.attrib.get('id')
            if(node.find('./state') is not None):
                self.node_compound=node.attrib.get('id')
                self.compoundstates_.append(node)
        self.skeleton_all_compoundstates()


    def get_simplestates(self):
        simplestates=[]
        for node in self.root.findall('.//state'):
            node_state=node.attrib.get('id')
            if(node.find('./state') is  None  ):
                self.node_simple=node.attrib.get('id')
                self.simplestates_.append(node)
                simplestates.append(self.node_simple)
        self.skeleton_all_simplestates()













