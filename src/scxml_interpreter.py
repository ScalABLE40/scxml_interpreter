#!/usr/bin/env python
#
# Copyright 2015 Airbus
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
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
from skill_class import StateMachine
import smach

# RE
RE_PREFIXED_TAG = "$"
RE_PREFIXED_BEGIN_TAG = "${"
RE_PREFIXED_END_TAG = "}"


def get_pkg_dir_from_prefix(path, ros_package_path=None):
    if RE_PREFIXED_TAG not in path:
        return path

    splitpath = path.split(RE_PREFIXED_END_TAG)
    after_prefix_path = splitpath[-1]
    prefix = splitpath[0].split(RE_PREFIXED_BEGIN_TAG)[-1]
    rpath = get_pkg_dir(prefix) + after_prefix_path

    return rpath



class SCXMLInterpreter(object):
    def __init__(self, scxml_file = None):
        self.StateDict = None
        file_ = scxml_file.read()
        file_ = re.sub(' xmlns="[^"]+"', '', file_, count=1)
        self.root = etree.fromstring(file_)
        self.skillProvider = None
        self.CheckBool = True

        self._next_parent_list = []
        self._current_parent_list = []
        self._next_SM_list = []
        self._current_SM_list = []

        if (self.root is None):
            rospy.logerr("[SCXML Interpreter] : Error in root !!!!")
            self.CheckBool = False
        else:
            pass

    def convertSCXML(self):
        '''
            First we import the skills
            Then we construct the state machine
            We return the constructed state machine
        '''

        skill = self.root.find("datamodel/data[@id='skill_file']")
        if (skill is None):
            rospy.logerr("[SCXML Interpreter] No Skill Register found !!")
            self.CheckBool = False
            return
        else:
            try:
                rospy.loginfo(str(skill.attrib['expr']))
                self.skillProvider = SkillProvider(get_pkg_dir_from_prefix(str(skill.attrib['expr'])))
            except Exception as ex:
                rospy.logerr(ex)
                rospy.logerr('[SCXML Interpreter] Skill register XML not found.')
                self.CheckBool = False

        return self.constructSM()

    def constructSM(self):
        '''
            Construct the mainStateMachine then all the state by recursive level construction (0, then all level 1, then all level 2)
        '''
        # Find the outcomes of the main state machine
        outcomes_ = []
        final_states = self.get_final_states_id(self.root)
        if (final_states == None):
            return
        else:
            ##Get all possible outcomes but still checking if the transition is consistent
            for final_id in final_states:
                for transition in self.root.findall("./*/transition[@target='" + str(final_id) + "']"):
                    event = transition.attrib.get('event')
                    if (event is None):
                        rospy.logerr("[SCXML Interpreter] : No Transition Event for %s !!!!" % transition)
                        self.CheckBool = False
                        return
                    target = transition.attrib.get('target')
                    if (target is None):
                        rospy.logerr("[SCXML Interpreter] : No Transition Target for %s !!!!" % transition)
                        self.CheckBool = False
                        return
                    if (target == final_id):
                        target = event
                    outcomes_.append(event)
        # Init the new smach state machine
        SM = StateMachine(outcomes = outcomes_)

        datamodel = self.root.find('datamodel')

        if (datamodel is None):
            rospy.logwarn("[SCXML Interpreter] : No Datamodel detected !!!!")
        else:
            for data in datamodel:
                data_ID = data.attrib['id']
                data_expr = data.attrib['expr']
                SM.userdata[data_ID] = data_expr

        ##Setupt the initial state for the intial state machine
        if (self.root.attrib.get('initial') == None):
            rospy.logerr("[SCXML Interpreter] : No Initial State in the SCXML !!!!")
            self.CheckBool = False
            return

        SM._initial_state_label = self.root.attrib.get('initial')
        ##Init variables for interations
        finish_ = False
        self._current_parent_list = ['.']
        self._current_SM_list = [SM]

        ##Main loop
        while (finish_ == False):
            ##Construct the actual level

            self._next_parent_list = []
            self._next_SM_list = []
            for i_parent in range(len(self._current_parent_list)):
                parent = self._current_parent_list[i_parent]
                current_SM = self._current_SM_list[i_parent]
                # open the current SM for construction
                current_SM.open()
                ##Check for final states if not a parallel state

                final_states = self.get_final_states_id(self.root.find(parent))
                if (final_states is None):
                    self.CheckBool = False
                    try:
                        current_SM.check_consistency()
                    except Exception as ex:
                        raise ex
                    current_SM.close()
                    return None
                # List all states and all parallel states
                list_state = self.root.findall(parent + '/state')
                #list_parallel = self.root.findall(parent + '/parallel')

                for state in list_state:
                    if(state.find('state') is not None):
                        self.constructCompoundState(state, parent, current_SM, SSM, final_states)
                    else:
                        self.constructSimpleState(state, current_SM, SM, final_states)
                    if(self.CheckBool == False):
                        try:
                            current_SM.check_consistency()
                        except Exception as ex:
                            raise ex
                        current_SM.close()
                        return None

                try:
                    current_SM.check_consistency()
                except Exception as ex:
                    raise ex
                current_SM.close()

            if (len(self._next_parent_list) == 0):  ##There is no lower level
                finish_ = True
            else:
                self._current_parent_list = self._next_parent_list  ##copy the new list of parent
                self._current_SM_list = self._next_SM_list  ##copy the list of State Machine

        if (self.CheckBool == False):
            return None
        else:
            return SM

    def constructCompoundState(self, current_level, parent, current_openSM, mainSM, final_states):
        ##State is parent ==> it's a state machine
        ID = current_level.attrib.get('id')

        if (ID is None):
            rospy.logerr("[SCXML Interpreter] No ID for a State !!!!")
            self.CheckBool = False
            return
            ##get onEntry and onExit
        onEntry = self.get_on_entry(current_level)
        onExit = self.get_on_exit(current_level)
        ##add to the parent list
        self._next_parent_list.append(parent + "/state[@id='" + str(ID) + "']")
        ##Find the transitions
        transitions_ = {}
        outcomes_ = []
        ##If the current_openSM is not a concurence then we can map the outcome, otherwise we have to map on this level
        ##Because there is no final state for a concurence and he can not transition to another state
        if (type(current_openSM) is not ssm_concurrence.ssmConcurrence):
            for transition in current_level.findall('transition'):
                event = transition.attrib.get('event')
                if (event is None):
                    rospy.logerr("[SCXML Interpreter] No Transition Event for %s !!!!" % ID)
                    self.CheckBool = False
                    return
                target = transition.attrib.get('target')
                if (target is None):
                    rospy.logerr("[SCXML Interpreter] No Transition Target for %s !!!!" % ID)
                    self.CheckBool = False
                    return
                ##if target a final state send we set on the outcome
                for final_id in final_states:
                    if (target == final_id):
                        target = event

                outcomes_.append(event)
                transitions_[event] = target
        else:
            ##remplace the final states of the parent to this level
            final_states = self.get_final_states_id(current_level)
            if (final_states == None):
                rospy.logerr("[SCXML Interpreter] No final State found in " + ID + " !!!!")
                self.CheckBool = False
                return
            ##if target a final state send we set on the outcome
            for final_id in final_states:
                for transition in current_level.findall("./state/transition[@target='" + str(final_id) + "']"):
                    event = transition.attrib.get('event')
                    if (event is None):
                        rospy.logerr("[SCXML Interpreter] No Transition Event for %s !!!!" % ID)
                        self.CheckBool = False
                        return
                    target = transition.attrib.get('target')
                    if (target is None):
                        rospy.logerr("[SCXML Interpreter] No Transition Target for %s !!!!" % ID)
                        self.CheckBool = False
                        return

                    outcomes_.append(event)
                    transitions_[event] = target

        ##Add the datamodel
        datamodel_ = self.get_datamodel(current_level, mainSM, ID)
        if (datamodel_ is None):
            return
            ##Find the io_keys
        keys_ = self.findIOkeys(current_level, ID)
        if (keys_ is None):
            return
        ##Add the intial state
        initial = current_level.find("initial")
        if (initial is None):
            rospy.logerr("[SCXML Interpreter] No Initial state for a Parent State !!!!")
            self.CheckBool = False
            return

        newSM = ssm_state_machine.ssmStateMachine(outcomes_, input_keys=keys_,
                                                  output_keys=keys_)  ##Define the empty state machine
        newSM._datamodel = datamodel_
        newSM._onEntry = onEntry
        newSM._onExit = onExit

        newSM._initial_state_label = str(initial.find('transition').attrib.get('target'))
        self.addToSM(current_openSM, ID, newSM, transitions_, keys_)
        self._next_SM_list.append(newSM)




    def constructSimpleState(self, current_level, current_openSM, mainSM, final_states):

        ID = current_level.attrib.get('id')

        if (ID is None):
            rospy.logerr("[SCXML Interpreter] No ID for a State !!!!")
            self.CheckBool = False
            return
        ##Find the transitions
        transitions_ = {}
        for transition in current_level.findall('transition'):
            event = transition.attrib.get('event')
            if (event is None):
                rospy.logerr("[SCXML Interpreter] No Transition Event for %s !!!!" % ID)
                self.CheckBool = False
                return
            target = transition.attrib.get('target')
            if (target is None):
                rospy.logerr("[SCXML Interpreter] No Transition Target for %s !!!!" % ID)
                self.CheckBool = False
                return
            ##if target a final state send we set on the outcome
            for final_id in final_states:
                if (target == final_id):
                    target = event

            transitions_[event] = target
        datamodel_ = self.get_datamodel(current_level, mainSM, ID)

        ##Find the skill
        skill_name = current_level.find("./datamodel/data[@id='skill']")
        if (skill_name is not None):
            if ('expr' in skill_name.attrib):
                try:
                    State_ref = self.skillProvider.load(skill_name.attrib.get('expr'))
                    State = State_ref()
                except Exception as ex:
                    self.CheckBool = False
                    return
            else:
                rospy.logerr('[SCXML Interpreter] Skill "expr" not defined in state "%s" !' % ID)
                self.CheckBool = False
                return
        else:
            rospy.logwarn("[SCXML Interpreter] No skill found for the state : %s. We use the EmptyState." % ID)
            State = ssm_state.EmptyState()

        keys_ = self.findIOkeys(current_level, ID)
        if (keys_ is None):
            return

        State.register_io_keys(keys_)
        State._datamodel = datamodel_
        self.addToSM(current_openSM, ID, State, transitions_, keys_)
        # Add the io_keys to the mainSM
        for keys in State.get_registered_input_keys():
            if (not (mainSM.userdata.__contains__(keys))):
                mainSM.userdata.__setattr__(keys, None)



    def get_final_states_id(self, current_level):
        final_states = current_level.findall('./final')
        final_states_id = []
        if (len(final_states) == 0):
            rospy.logerr("[SCXML Interpreter] No final State found for the state : %s" % str(current_level))
            self.CheckBool = False
            return None
        for state in final_states:
            final_id = state.attrib.get('id')
            if (final_id == None):
                rospy.logerr("[SCXML Interpreter] No id for the final state : %s" % str(state))
                self.CheckBool = False
                return None
            else:
                final_states_id.append(final_id)

        return final_states_id

    def get_datamodel(self, state, mainSM, ID):
        datamodel_ = {}
        for data in state.findall('./datamodel/data'):
            data_ID = data.attrib['id']
            if ('expr' in data.attrib):
                data_expr = data.attrib['expr']
            else:
                rospy.logwarn(
                    '[SCXML Interpreter] State : "%s". "expr" not defined for data "%s". It will be set to \"\"!' % (
                    ID, data_ID))
                data_expr = ""
            datamodel_[data_ID] = data_expr
            if (not (mainSM.userdata.__contains__(data_ID))):
                mainSM.userdata.__setattr__(data_ID, data_expr)
        return datamodel_

    def findIOkeys(self, state, ID):
        io_keys = set()
        for data in state.findall('.//datamodel/data'):
            ##Grab all data ID
            io_keys.add(data.attrib['id'])
            ##Grad all skills
            if (data.attrib['id'] == 'skill'):
                if ('expr' in data.attrib):
                    try:
                        State = self.skillProvider.load(data.attrib['expr'])()
                    except Exception as ex:
                        rospy.logerr('[SCXML Interpreter] Import fail from Skill "%s" in state : "%s"!' % (
                        data.attrib['expr'], ID))
                        rospy.logerr(ex)
                        return None
                else:
                    rospy.logerr('[SCXML Interpreter] Skill "expr" not defined in state "%s" !' % ID)
                    return None
            else:
                State = ssm_state.EmptyState()
            for key in State.get_registered_input_keys():
                io_keys.add(str(key))

        for log in state.findall('.//onentry/log'):
            ##Grab all data ID
            if (not (log.attrib['label'] == "") and not (log.attrib['label'] == 'outcome')):
                io_keys.add(log.attrib['label'])
        for log in state.findall('.//onexit/log'):
            ##Grab all data ID
            if (not (log.attrib['label'] == "") and not (log.attrib['label'] == 'outcome')):
                io_keys.add(log.attrib['label'])

        return list(io_keys)


    def addToSM(self,current_SM,added_ID, added_State, transitions_,keys_):
        '''
        Add a state, a state machine or a concurence to the current state machine / concurence
        '''
        remapping_ = {'logfile':'logfile'}
        for keys in keys_:
            remapping_[keys] = keys
        if(type(current_SM) is StateMachine):
            current_SM.add(added_ID,added_State,transitions_,remapping_)
