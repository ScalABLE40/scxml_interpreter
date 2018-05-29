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
        self.simplestates=[]
        self.compoundstates=[]
        self.finalstates_=[]
        self.parallelstates=[]
        preprocess_mapping = {}
##parsing the file correctly####
    def parsing_scxml(self,scxml_file):
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
        compoundstates=[]
        for node in self.compoundstates:
            if(node is not None):
                compoundstates.append(self.skeleton_compoundstate(node))
        return compoundstates

######Each compound state provides all info state,transitions######
    def skeleton_compoundstate(self,node):
        states=[]
        node_id_compound=node.attrib.get('id')
        initial=node.find('./initial/transition').attrib.get('target')
        if(node_id_compound is not None):
            transition=self.get_transition(node)
            datamodel=self.get_datamodel(node)
            onEntry = self.get_on_entry(node)
            onExit = self.get_on_exit(node)

        else:
            rospy.logerr("[SCXML Interpreter] No ID found!!!!")
        for state in node.findall('./state'):
            test=state.attrib.get('id')
            states.append(test)
        skeleton = CompoundStateSkeleton(node_id_compound,datamodel, transition,states,initial,onEntry,onExit)
        return skeleton

#########creating all the simple states in the scxml######
    def skeleton_all_simplestates(self):
      simplestates=[]
      for node in self.simplestates:
            if(node is not None):
                simplestates.append(self.skeleton_simplestate(node))
      return simplestates
######Each simple state provides all info state,transitions######
    def skeleton_simplestate(self,node):
        node_id=node.attrib.get('id')
        onEntry = self.get_on_entry(node)
        onExit = self.get_on_exit(node)
        if(node_id is not None):
            transition=self.get_transition(node)
            datamodel=self.get_datamodel(node)

        skeleton = SimpleStateSkeleton(node_id,datamodel,transition,onEntry,onExit)
        return skeleton
###############Parallel states####################
    def skeleton_all_parallelstates(self):
      parallelstates=[]
      for node in self.parallelstates:
            if(node is not None):
                parallelstates.append(self.skeleton_parallelstate(node))
      return parallelstates

    def skeleton_parallelstate(self,node):
        states=[]
        node_id=node.attrib.get('id')
        initial=node.find('./initial')
        if(node_id is not None):
            transition=self.get_transition_parallel(node)
            datamodel=self.get_datamodel(node)
            finalstates=self.get_final_states_id()
            onEntry = self.get_on_entry(node)
            onExit = self.get_on_exit(node)
        for state in node.findall('./state'):
            test=state.attrib.get('id')
            states.append(test)
        skeleton = ParallelStateSkeleton(node_id,datamodel, transition,states,initial,onEntry,onExit)
        return skeleton


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
            self.get_compoundstates()
            self.get_simplestates()
            self.get_parallelstates()
        else:
            rospy.logerr("[SCXML Interpreter] could not find initial state")
        final_states=self.get_final_states_id()
        data=self.get_datamodel(root)
        skeleton=RootStateSkeleton(initial,data,final_states,states)
        return skeleton


#######transitions details for simple and compound states###############
    def get_transition(self,current_state):
        current=current_state.attrib.get('id')
        transitions_={}
        target=[]
        outcomes_=[]
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

###Transitions for parallel states#####
    def get_transition_parallel(self,current_state):
       current=current_state.attrib.get('id')
       transitions_={}
       target=[]
       outcomes_=[]
       outcomes_map = {}
       for transition in current_state.findall('transition'):
           event = transition.attrib.get('event')
           if(event is None):
               rospy.logerr("[SCXML Interpreter] No Transition Event for %s !")
               return
           target = transition.attrib.get('target')
           if(target is None):
               rospy.logerr("[SCXML Interpreter] No Transition Target for %s !!!!")
               return
           cond = transition.attrib.get('cond')
           if(target is None):
               rospy.logerr("[SCXML Interpreter] No Transition Condition  for %s !!!!")
               return
           map_list = self.convertToConcurenceMap(cond)   
           if(len(map_list) == 0):
               rospy.logerr("[SCXML Interpreter] Error during the convertion event to map for %s !!!!")
               return

           for map_ in map_list:
               outcome_ = event
               outcomes_.append(outcome_)
               outcomes_map[outcome_] = map_
               transitions_[outcome_] = target
       return transitions_

    def convertToConcurenceMap(self, event):
        map_list = []
        if(event.find(' AND ') == -1): ##There is no "and" condition
            if(event.find(' OR ') == -1): ## There is no "or" condition
                list_ = event.split('.')
                map_list.append({list_[0]:list_[1]})
            else:
                list_or = event.split(' OR ')
                for event_or in list_or:
                    list_ = event_or.split('.')
                    map_list.append({list_[0]:list_[1]})
        else:
            list_and = event.split(' AND ')
            map_ = {}
            for event_and in list_and:
                list_and = event_and.split('.')
                map_[list_and[0]] = list_and[1]
            map_list.append(map_)
        return map_list

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

 ####get states#######
    def get_compoundstates(self):
        compoundstates=[]
        for node in self.root.findall('.//state'):
            node_state=node.attrib.get('id')
            if(node.find('./state') is not None):
                self.node_compound=node.attrib.get('id')
                self.compoundstates.append(node)
        self.skeleton_all_compoundstates()
        return self.compoundstates


    def get_simplestates(self):
        simplestates=[]
        for node in self.root.findall('.//state'):
            node_state=node.attrib.get('id')
            if(node.find('./state') is  None  ):
                self.node_simple=node.attrib.get('id')
                self.simplestates.append(node)
                simplestates.append(self.node_simple)
        self.skeleton_all_simplestates()
        return self.simplestates


    def get_parallelstates(self):
        parallelstates=[]
        #rospy.loginfo("parallel %s"%parallelstates)
        for node in self.root.findall('parallel'):
            node_state=node.attrib.get('id')
            parallelstates.append(node)

        self.skeleton_all_parallelstates()
        return self.parallelstates
###OnEntry####
    def get_on_entry(self, state):
        if(state.find('onentry') == None):
            return []
        else:
            '''
            logs = {}
            script = None
            if(state.find('./onentry/script') is not None):
                script = state.find('./onentry/script').text

            for log in state.findall('./onentry/log'):
                logs[log.attrib['label']] = log.attrib['expr']
            '''
            return []
        
###OnExit#####
    def get_on_exit(self, state):
        if(state.find('onexit') == None):
            return []
        else:
            '''
            logs = {}
            script = None
            if(state.find('./onexit/script') is not None):
                script = state.find('./onexit/script').text

            for log in state.findall('./onexit/log'):
                logs[log.attrib['label']] = log.attrib['expr']
            '''
            return []






