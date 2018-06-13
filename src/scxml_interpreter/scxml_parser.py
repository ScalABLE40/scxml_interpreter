#!/usr/bin/env python

import rospy
import re
from roslib.packages import get_pkg_dir
import xml.etree.ElementTree as etree
import smach
import sys
from os import path
import os
import traceback

from scxml_interpreter.interface_class import *
from scxml_interpreter.Errorexecptions import *

class SCXMLParser:
    def __init__(self):
        self.isfile=True
        self.simplestates=[]
        self.compoundstates=[]
        self.finalstates_=[]
        self.parallelstates=[]
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
                self.root_Interface(self.root)
        except ParseError as ex:
            rospy.logerr(ex)
            rospy.logerr('parsing is not correct is not SCXML')

        return self.root
#########creating all the compound states in the scxml######
    def Interface_all_compoundstates(self):
        compoundstates=[]
        for node in self.compoundstates:
            if(node is not None):
                compoundstates.append(self.Interface_compoundstate(node))
        return compoundstates

######Each compound state provides all info state,transitions######
    def Interface_compoundstate(self,node):
        states=[]
        node_id_compound=node.attrib.get('id')
        initial=node.find('./initial/transition').attrib.get('target')
        if(node_id_compound is not None):
            transition=self.get_transition(node)
            datamodel=self.get_datamodel(node)
            onEntry = self.get_on_entry(node)
            onExit = self.get_on_exit(node)

        else:
            raise InterpreterError(node_id="[SCXML Interpreter] No ID found!!!! %s"%node_id_compound)
        for state in node.findall('./state'):
            test=state.attrib.get('id')
            states.append(test)
        Interface = CompoundStateInterface(node_id_compound,datamodel, transition,states,initial)
        #print Interface
        return Interface

#########creating all the simple states in the scxml######
    def Interface_all_simplestates(self):
      simplestates=[]
      for node in self.simplestates:
            if(node is not None):
                simplestates.append(self.Interface_simplestate(node))
      return simplestates
######Each simple state provides all info state,transitions######
    def Interface_simplestate(self,node):
        node_id=node.attrib.get('id')
        onEntry = self.get_on_entry(node)
        onExit = self.get_on_exit(node)
        if(node_id is not None):
            transition=self.get_transition(node)
            datamodel=self.get_datamodel(node)
        Interface = SimpleStateInterface(node_id,datamodel,transition,onEntry,onExit)
        print Interface
        return Interface
###############Parallel states####################
    def Interface_all_parallelstates(self):
      parallelstates=[]
      for node in self.parallelstates:
            if(node is not None):
                parallelstates.append(self.Interface_parallelstate(node))
      return parallelstates

    def Interface_parallelstate(self,node):
        states=[]
        node_id=node.attrib.get('id')
        initial=node.find('./initial')
        onEntry = self.get_on_entry(node)
        onExit = self.get_on_exit(node)
        if(node_id is not None):
            transition=self.get_transition_parallel(node)
            datamodel=self.get_datamodel(node)
            finalstates=self.get_final_states_id()

        for state in node.findall('./state'):
            test=state.attrib.get('id')
            states.append(test)
        Interface = ParallelStateInterface(node_id,datamodel, transition,states,onEntry,onExit)
        print Interface
        return Interface


    def root_Interface(self,root):
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
            raise(InterpreterError(initial_state="[SCXML Interpreter] No inital state found!!!! %s"%initial))
        final_states=self.get_final_states_id()
        data=self.get_datamodel(root)
        Interface=RootStateInterface(initial,data,final_states,states)
        return Interface


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
                    raise (InterpreterError(event="[SCXML Interpreter] No Transition Event for %s !!!!"%event))
                target_state = transition.attrib.get('target')
                if(target_state is None):
                    raise (InterpreterError(target="[SCXML Interpreter] No Transition Target for %s !!!!"%target_state ))
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
                raise (InterpreterError(event="[SCXML Interpreter] No Transition Event for %s !"%event))
           target = transition.attrib.get('target')
           if(target is None):
                raise (InterpreterError(target="[SCXML Interpreter] No Transition Target for %s !!!!"%target))
           cond = transition.attrib.get('cond')
           if(target is None):
                raise (InterpreterError(condition="[SCXML Interpreter] No Transition Condition  for %s !!!!"%condition))
           map_list = self.convertToConcurenceMap(cond)
           if(len(map_list) == 0):
               raise (InterpreterError(map="[SCXML Interpreter] Error during the convertion event to map for %s !!!!"%map_list))
           for map_ in map_list:
               outcome_ = event
               outcomes_.append(outcome_)
               outcomes_map[outcome_] = map_
               transitions_[outcome_] = target
       return transitions_,outcomes_map

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
                raise (InterpreterError("[SCXML Interpreter] Data is empty"))
        return datamodel_

########final state#############
    def get_final_states_id(self):
        final_states = self.root.findall('./final')
        final_states_id = []
        if(len(final_states) == 0):
            raise (InterpreterError("[SCXML Interpreter] No final State found for the state"))
        for state in final_states:
            final_id = state.attrib.get('id')
            if(final_id == None):
                raise (InterpreterError("[SCXML Interpreter] No id for the final state "))
        return final_states_id

 ####get states#######
    def get_compoundstates(self):
        compoundstates=[]
        for node in self.root.findall('.//state'):
            node_state=node.attrib.get('id')
            if(node.find('./state') is not None):
                self.node_compound=node.attrib.get('id')
                self.compoundstates.append(node)
        self.Interface_all_compoundstates()
        return self.compoundstates


    def get_simplestates(self):
        simplestates=[]
        for node in self.root.findall('.//state'):
            node_state=node.attrib.get('id')
            if(node.find('./state') is  None  ):
                self.node_simple=node.attrib.get('id')
                self.simplestates.append(node)
                simplestates.append(self.node_simple)
        self.Interface_all_simplestates()
        return self.simplestates


    def get_parallelstates(self):
        parallelstates=[]
        #rospy.loginfo("parallel %s"%parallelstates)
        for node in self.root.findall('parallel'):
            node_state=node.attrib.get('id')
            self.parallelstates.append(node)
        self.Interface_all_parallelstates()
        print parallelstates
        return self.parallelstates

###OnEntry####
    def get_on_entry(self,state):
        if(state.find('onentry') == None):
            return {}
        else:
            script_attributes={}
            script={}
            scripts=state.findall('./onexit/script')
            if(state.find('./onentry/script') != None):
                if(state.find('./onentry/script').text is not None or state.find('./onentry/script').attrib.get('src') is not  None):
                    if(len(scripts)>1):
                        raise (InterpreterError("[SCXML Parser] More than one script in On entry"))
                    else:
                        content = state.find('./onentry/script').text
                        script_attributes['content']=content
                        for node in state.findall('./onentry/script'):
                            src=node.attrib.get('src')
                            script_attributes['src']=src
                            script['script']=script_attributes
        return script

###On exit###
    def get_on_exit(self,state):
        if(state.find('onexit') == None):
            return {}
        else:
            node_id=None
            script_attributes={}
            script={}
            scripts=state.findall('./onexit/script')
            if(state.find('./onexit/script') != None):
                if(state.find('./onexit/script').text is not None or state.find('./onexit/script').attrib.get('src') is not None):
                    if(len(scripts)>1):
                        raise (InterpreterError("[SCXML Parser] More than one script in On exit"))
                    else:
                        content = state.find('./onexit/script').text
                        script_attributes['content']=content
                        for node in state.findall('./onexit/script'):
                            src=node.attrib.get('src')
                            script_attributes['src']=src
                            script['script']=script_attributes
        return script
