#!/usr/bin/env python
#
# Copyright 2015 Airbus
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the A(object)pache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


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

class SCXMLInterpreter:
    def __init__(self, scxml_file):
        self.isfile=True
        self.simplestates_=[]
        self.compoundstates_=[]
        self.finalstates_=[]
        preprocess_mapping = {}
##pasring the file correctly####
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
                self.hierarchial_skeleton()
        except ParseError as ex:
            rospy.logerr(ex)
            rospy.logerr('parsing is not correct is not SCXML')


    def skeleton_compoundstate(self,root):
        root=self.root
        transitions_={}
        outcomes_=[]
        compound_traget=[]
        compoundstates=[]
        matches=[]
        datamodel_ = {}
        for states in self.root.findall('.//state'):
            if(states is not None):
                cstate=states.attrib.get('id')
                compoundstates.append(cstate)
            for stat in compoundstates:
                for state in self.compoundstates_:
                    if(state is not None and stat is not None):
                        if(state==stat):
                            matches.append(state)
###Get the event,target,data of datamodel fro the states which are only compound####
                            for cstates in self.root.findall(".//state[@id='"+str(stat)+"']"):
                                if(cstates is not None):
                                    for transition in cstates.findall("transition"):
                                        event = transition.attrib.get('event')
                                        if(event is None):
                                            rospy.logerr("[SCXML Interpreter] No Transition Event for %s !!!!")
                                            return
                                        target_state = transition.attrib.get('target')
                                        if(target_state is None):
                                            rospy.logerr("[SCXML Interpreter] No Transition Target for %s !!!!" )
                                            return
                                        compound_traget.append(target_state)
                                        transitions_[event] = target_state

##########Datamodel and data########################
                                    for data in cstates.findall('./datamodel/data'):
                                        if( data is not None):
                                            data_IDcompund = data.attrib['id']
                                            if('expr' in data.attrib):
                                                data_expr = data.attrib['expr']
                                            else:
                                                rospy.logwarn('no datamodel')
                                            datamodel_[data_IDcompund] = data_expr
                                        else:
                                            rospy.logerr("[SCXML Interpreter] Data is empty")
                                else:
                                    rospy.logerr("[SCXML Interpreter] could not find the matching compund state ID ")

        rospy.loginfo('data id %s'%datamodel_)
        rospy.loginfo('compound state  transitions%s'%transitions_)


    def skeleton_simplestate(self,root):
        root=self.root
        transitionssimple_={}
        outcomes_=[]
        traget=[]
        simstates=[]
        matches=[]
        datamodel_ = {}
        for simplestates in self.root.findall('.//state'):
            if(simplestates is not None):
                sstate=simplestates.attrib.get('id')
                simstates.append(sstate)
            for stat in simstates:
                for state in self.simplestates_:
##check if the states which has all states is equal to the list of simple states###

                    if(state==stat):
                        matches.append(state)
###Get the event,target,data of datamodel fro the states which are only simple####

                        for sstates in self.root.findall(".//state[@id='"+str(stat)+"']"):
                            if(sstates is not None):
                                for transition in sstates.findall("transition"):
                                    event = transition.attrib.get('event')
                                    if(event is None):
                                        rospy.logerr("[SCXML Interpreter] No Transition Event for %s !!!!" % ID)
                                        return
                                    target_simplestate = transition.attrib.get('target')
                                    if(target_simplestate is None):
                                        rospy.logerr("[SCXML Interpreter] No Transition Target for %s !!!!" % ID)
                                        return
                                    transitionssimple_[event] = target_simplestate
##########Datamodel and data########################
                                for data in sstates.findall('./datamodel/data'):
                                    if(data is not None):
                                        data_ID = data.attrib['id']
                                        if('expr' in data.attrib):
                                            data_expr = data.attrib['expr']
                                        else:
                                            rospy.logwarn('no datamodel')
                                        datamodel_[data_ID] = data_expr
                                    else:
                                        rospy.logerr("[SCXML Interpreter] Data is empty")
                            else:
                                rospy.logerr("[SCXML Interpreter] could not find the matching simple state ID ")
        rospy.loginfo('data id %s'%datamodel_)
        rospy.loginfo('simple state transition %s'%transitionssimple_)


    def hierarchial_skeleton(self):
        root=self.root
        self.current_states=[]
        initial=self.root.attrib.get('initial')
        rospy.loginfo('node is  %s'%initial)
        self._atomic_state_list = []
        self._parent_list=[]
        self.allstates=[]
        if(initial is not None):
            for state in self.root.findall('.//state'):
                for init in state.findall('./initial/transition'):
                    init_target=init.attrib.get('target')
                    event = init.attrib.get('event')
                    rospy.loginfo('init tagert %s'%init_target)
                    rospy.loginfo('event  %s'%event)

            for node in self.root.findall('.//state'):
                ID=node.attrib.get('id')

###check the inital state and the check the state is compound or simple state####

                if(initial==ID):
                    self.start_of_State=initial
                    if(node.find('./state') is not None  ):
                        node_cmpnd=node.attrib.get('id')
                        self.compoundstates_.append(node_cmpnd)
                        self.skeleton_compoundstate(root)
                    else:
                        node_simple=node.attrib.get('id')
                        self.simplestates_.append(node_simple)
                        self.skeleton_simplestate(root)
            for transition in self.root.findall('.//transition'):
                target_state=transition.attrib.get('target')
                self.allstates.append(target_state)
            for node in self.root.findall('.//state'):
                node_state=node.attrib.get('id')

####checking if the target and the state is same if it is same checking if it simple or compound state####

                for states in self.allstates:
                    if(node_state==states):
                        if(node.find('./state') is not None  ):
                            node_cmpnd=node.attrib.get('id')
                            self.compoundstates_.append(node_cmpnd)
                            rospy.loginfo('Compound State %s'%node_cmpnd)
                            self.skeleton_compoundstate(root)
                        else:
                            node_simple=node.attrib.get('id')
                            self.simplestates_.append(node_simple)
                            rospy.loginfo('Simple State%s'%node_simple)
                            self.skeleton_simplestate(root)




















