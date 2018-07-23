#!/usr/bin/env python

from state_provider import SmachStateProvider
import smach
import rospy
import ast


def convert_datamodel_to_userdata(ud, initial_state, *args):
    for data_id,data_value in args[0].iteritems():
        try:
            ud[data_id] = ast.literal_eval(data_value)
        except Exception as e:
            ud[data_id] = data_value

def transition_convertion(ud, actives_states, *args):
    if(args[0] in actives_states):
        convert_datamodel_to_userdata(ud, "", args[1])


class SmachBuilder():
    def __init__(self, interface, provider):
        self._interface = interface
        self._provider = provider
        self.states_instances = {}
        self.userdata = smach.UserData()

    def add_states_from_interface(self,sm_interface, open_sm, add_keys = False):
        with open_sm:
            for child_state in sm_interface.states:
                child_state_interface = self.find_matching_interface(child_state)
                open_sm.add(child_state_interface.id, self.states_instances[child_state_interface.id], child_state_interface.transitions)
                if(child_state_interface.id == sm_interface.initial_state_id):
                    open_sm.register_start_cb(convert_datamodel_to_userdata, [child_state_interface.data])
                else:
                    open_sm.register_transition_cb(transition_convertion, [child_state_interface.id, child_state_interface.data])
                if(add_keys):
                    for keys in set(self.states_instances[child_state_interface.id].get_registered_output_keys()).union(set(self.states_instances[child_state_interface.id].get_registered_input_keys())):
                        open_sm.register_io_keys([keys])

        open_sm.set_initial_state([sm_interface.initial_state_id])
        if(sm_interface.data):
            open_sm.register_start_cb(convert_datamodel_to_userdata,[sm_interface.data])
        for i_key in set(open_sm.get_registered_output_keys()).union(set(open_sm.get_registered_input_keys())):
            self.userdata[i_key] = None

    def create_root_state_machine(self):
        rootInterface = self._interface.rootState
        root_SM = smach.StateMachine(rootInterface.final_states)
        self.add_states_from_interface(rootInterface, root_SM)
        root_SM.userdata = self.userdata
        return root_SM

    def create_all_compound_states(self):
        states = {}
        for stateInterface in self._interface.compoundStates:
            id, state = self.create_compound_state(stateInterface)
            states[id] = state
        return states

    def find_matching_interface(self, id):
        for interface in self._interface.simpleStates:
            if(interface.id == id):
                return interface
        for interface in self._interface.compoundStates:
            if(interface.id == id):
                return interface

    def create_compound_state(self, stateInterface):
        #PROVIDER 
        state = smach.StateMachine(stateInterface.get_outcomes())
        self.add_states_from_interface(stateInterface, state, True)
        return stateInterface.id, state

    def create_all_simple_states(self):
        states = {}
        for stateInterface in self._interface.simpleStates:
            id, state = self.create_simple_state(stateInterface)
            states[id] = state
        return states

    def get_python_state_automatch(self, stateInterface):
            ##Use Auto matched
        matching_states = []
        for provider in self._provider:
            match_state = provider.get_state(stateInterface.id)
            if(match_state is not None):
                matching_states.append(match_state)
        
        if(len(matching_states) == 0):
            ##No state found in the registered provider
            ##Raise Error Code
            raise NameError("No State matching name found for the %s state. Please check Providers."%(stateInterface.id))
        elif(len(matching_states) > 1):
            ## more than one matching
            raise NameError("More than one state matching name found for the %s state. Please consider making it explicit in the SCXML file.")
        else:
            ##Match found
             return matching_states[0]

    def get_python_state(self, stateInterface):
        found_state = None
        if("python_state" in stateInterface.data):
            for provider in self._provider:
                found_state = provider.get_state(stateInterface.data["python_state"])
                rospy.logdebug("State '%s' linked to '%s' python state set."%(stateInterface.id, stateInterface.data["python_state"]))
            if found_state == None:
                #No matching found for the user define state
                raise NameError("The linked Python State '%s' for the State '%s' was not found. Please check Providers.")
        else:
            found_state = self.get_python_state_automatch(stateInterface)
        
        if(found_state is not None):
            if not(isinstance(found_state, smach.State)):
                ##Not a smachState for the smach  builder
                raise TypeError("State '%s' linked Python State is not Smach State instances"%(stateInterface.id))
            else:
                return found_state

    def create_simple_state(self, stateInterface):
        #PROVIDER
        ##Use define matching
        print(stateInterface)
        state = self.get_python_state(stateInterface)
        outcomes = list(state.get_registered_outcomes())
        for outcome in outcomes:
            if not(outcome in stateInterface.get_outcomes()):
                ##Raise error
                raise NameError("Outcome '%s' from the Python class associate with the State '%s' is not defined in the SCXML."%(outcome, stateInterface.Id))
        for outcome in stateInterface.get_outcomes():
            if not(outcome in outcomes):
                ##Raise error
                raise NameError("Outcome '%s' defined in the SCXML State '%s' is not defined in is linked Python class."%(outcome, stateInterface.Id))
        return stateInterface.id, state
    
    def build_state_machine(self):
        simplestates = self.create_all_simple_states()
        self.states_instances = simplestates
        compoundstates = self.create_all_compound_states()
        states_instances = compoundstates.copy()
        self.states_instances.update(states_instances)
        return self.create_root_state_machine()