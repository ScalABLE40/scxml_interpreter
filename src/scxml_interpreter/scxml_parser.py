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
    """Parsing all the details for constructing the state machine from scxml machine"""
    def parsing_scxml(self,scxml_file):
        """file open  and file check if the file is scxml or notself.
        Converting the SCXML data to python structure.Here it is Interface

        Args:
            param1(scxml format):scxml_file

        Returns:
            Object:python structured SCXML data
        """
        try:
            ParseError = etree.ParseError
        except ImportError:
            from xml.parsers import expat
            ParseError = expat.ExpatError
        try:

            if os.path.splitext(scxml_file)[1] == ".scxml":
                file_ = open(scxml_file).read()
                file_ = re.sub(' xmlns="[^"]+"', '', file_, count=1)
                self.root = etree.fromstring(file_)
                self.root_Interface(self.root)
        except ParseError as ex:
            rospy.logerr(ex)
            rospy.logerr('Parsing is not correct and the file is not SCXML')

        SCXMLinterface = SCXMLInterface()
        SCXMLinterface.rootState = self.root_Interface(self.root)
        SCXMLinterface.simpleStates =   self.get_simplestates()
        SCXMLinterface.compoundStates = self.get_compoundstates()
        SCXMLinterface.parallelstates=self.get_parallelstates()
        return SCXMLinterface

    def Interface_all_compoundstates(self,compoundstates):
        """Interface of all the compound states in present in scxml file

        Here it will provide all compound states refernce by havinf for loopself.

        Args:
            param1 (list): compoundstates.
        Returns:
            list: The return Object reference of all compound states in SCXML file
        """
        compoundstates_interface=[]
        for node in compoundstates:
            if(node is not None):
                compoundstates_interface.append(self.Interface_compoundstate(node))
        return compoundstates_interface

######Each compound state provides all info state,transitions######
    def Interface_compoundstate(self,node):
        """Interface of each compound states is present from scxml file

        Here it will provide all attributes necessary to build the State machine
        Attributes of a Compound state:
        1.Node id---the id(i.e Name of the state) of the compound state
        2.Initial---The initial state of the state machine
        3.Transition---the transtion between two states has event,target
        when event is trigerred there will be a transit from current state to the target state
        4.Datamodel---userdata inputs
        5.finalStates=final states of the compound state ends

        Args:
            param1 (list): node.which is the current node
        Returns:
            list: The return all the details of  compound states in the python structure format
        """
        states=[]
        node_id_compound=node.attrib.get('id')
        initial=node.find('./initial/transition').attrib.get('target')
        if(node_id_compound is not None):
            transition=self.get_transition(node)
            datamodel=self.get_datamodel(node)
            onEntry = self.get_on_entry(node)
            onExit = self.get_on_exit(node)
            finalstates=self.get_final_states_id()
        else:
            raise InterpreterError(node_id="[SCXML Interpreter] No ID found!!!! %s"%node_id_compound)
        for state in node.findall('./state'):
            test=state.attrib.get('id')
            states.append(test)
        Interface = CompoundStateInterface(node_id_compound,datamodel, transition,states,initial,onEntry,onExit,)
        return Interface

    def Interface_all_simplestates(self, simplestates):
      """
      Interface of all the simple states are present in scxml file

      Here it will provide all simple states reference by having for loops

        Args:scxml_parser.py,
            param1 (list): simplestates.
        Returns:
            list: The return Object reference of all simple states in SCXML file
      """
      simplestates_interface=[]
      for node in simplestates:
            if(node is not None):
                simplestates_interface.append(self.Interface_simplestate(node))
      return simplestates_interface

    def Interface_simplestate(self,node):
        """Interface of each simple states is present from scxml file

        Here it will provide all attributes necessary to build the State machine
        Attributes of a Compound state:
        1.Node id---the id(i.e Name of the state) of the simple state
        2.Initial---The initial state of the state machine
        3.Transition---the transtion between two states has event,target
        when event is trigerred there will be a transit from current state to the target state
        4.Datamodel---userdata inputs
        Args:
            param1 (list): node.which is the current node
        Returns:
            list: The return all the details of  simple states in the python structure format
        """
        node_id=node.attrib.get('id')
        onEntry = self.get_on_entry(node)
        onExit = self.get_on_exit(node)
        if(node_id is not None):
            transition=self.get_transition(node)
            datamodel=self.get_datamodel(node)
        Interface = SimpleStateInterface(node_id,datamodel,transition,onEntry,onExit)
        return Interface

    def Interface_all_parallelstates(self,parallelstates):
      """
      Interface of all parallel states are present in scxml file

      Here it will provide all parallel states reference by looping parallelstates_interface
      """
      parallelstates_interface=[]
      for node in parallelstates:
            if(node is not None):
                parallelstates_interface.append(self.Interface_parallelstate(node))
      return parallelstates_interface

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
        return Interface


    def root_Interface(self,root):
        """Interface of all the root state in present in scxml file

        From root the flow of state machine begins

        Args:
            param1 (list): root.
        Returns:
            list:
            1.Initial---The initial state of the state machine which is root of the state machine
            2.Data---userdata inputs
            3.finalStates=final states of the root state ends
            4.States=all the state which is connected to the root state
        """
        root=self.root
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
            for node in self.root.findall('state'):
                ID=node.attrib.get('id')
                states.append(ID)
        else:
            raise(InterpreterError(initial_state="[SCXML Interpreter] No inital state found!!!! %s"%initial))
        final_states=self.get_final_states_id()
        data=self.get_datamodel(root)
        Interface=RootStateInterface(initial,data,final_states,states)
        return Interface


    def get_transition(self,current_state):
        """This method is to get transition of the current state(simple staes and compound states) where we have event:target
        when the event is triggered there will be trantstion from one state to target state

        Args:
            param1(string) :current_state

        Returns:
            dict:transition{event:target}
        """
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

    def get_datamodel(self,current_data):
        """This method is to get datamodel of the current state data where we have id:expression
        when the event is triggered there will be trantstion from one state to target state

        Args:
            param1(string) :current_data

        Returns:
            dict:datamodel{data_id:data_expr}
        """
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

    def get_final_states_id(self):
        """This method is to get final state id's
        where each state have the end state
        Returns:
            list:final states
        """
        final_states = self.root.findall('./final')
        final_states_id = []
        if(len(final_states) == 0):
            raise (InterpreterError("[SCXML Interpreter] No final State found for the state"))
        for state in final_states:
            final_id = state.attrib.get('id')
            final_states_id.append(final_id)
            if(final_id == None):
                raise (InterpreterError("[SCXML Interpreter] No id for the final state "))
        return final_states_id


    def get_compoundstates(self):
        """This method is to get compound state id's where all the states are placed in the list
        and this list is used whereever necessary to get each element

        Returns:
            object:refernce of all compound states
        """
        compoundstates=[]
        for node in self.root.findall('.//state'):
            node_state=node.attrib.get('id')
            if(node.find('./state') is not None):
                self.node_compound=node.attrib.get('id')
                compoundstates.append(node)
        return self.Interface_all_compoundstates(compoundstates)


    def get_simplestates(self):
        """This method is to get simple state id's where all the states are placed in the list
        and this list is used whereever necessary to get each element

        Returns:
            object:refernce of all simple states
        """
        simplestates = []
        for node in self.root.findall('.//state'):
            node_state=node.attrib.get('id')
            if(node.find('./state') is  None  ):
                self.node_simple=node.attrib.get('id')
                simplestates.append(node)
        return self.Interface_all_simplestates(simplestates)

    def get_parallelstates(self):
        """This method is to get parallel state id's where all the states are placed in the list
        and this list is used whereever necessary to get each element
        Returns:
            object:refernce of all parallel states
        """
        parallelstates=[]
        for node in self.root.findall('parallel'):
            node_state=node.attrib.get('id')
            parallelstates.append(node)
        return self.Interface_all_parallelstates(parallelstates)


    def get_on_entry(self,state):
        """This method is to get on entry
        A wrapper element containing executable content to be executed when the state is entered.
        Note:
        A conformant SCXML document MUST specify either the 'src' attribute or script content, but not both
        Args:
            param1(string):state
        Returns:
            dict:{src:script}
        """

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

    def get_on_exit(self,state):
        """This method is to get on entry
        A wrapper element containing executable content to be executed when the state is exited.
        Note:
        A conformant SCXML document MUST specify either the 'src' attribute or script content, but not both
        Args:
            param1(string):state
        Returns:
            dict:{src:script}
        """

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
