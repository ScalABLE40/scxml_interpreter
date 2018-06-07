#!/usr/bin/env python

from skill_provider import SkillProvider
import smach
import rospy
import ast

def convert_datamodel_to_userdata(ud, initial_state, *args):
    for data_id,data_value in args[0].iteritems():
        try:
            ud[data_id] = ast.literal_eval(data_value)
        except Exception as e:
            ud[data_id] = data_value 

class SmachBuilder():
    def __init__(self, Interface=None, skill_provider=None):
        self.Interface = Interface
        self.skill_provider = skill_provider
        self.root_SM = None
        self.states_instances = {}
        
    def create_root_state_machine(self):
        rootInterface = self.Interface.rootState
        self.root_SM = smach.StateMachine(rootInterface.final_states)
        with self.root_SM:
            for child_state in rootInterface.states:
                self.root_SM.add(child_state.id, self.states_instances[child_state.id], child_state.transitions)
                #for keys in self.states_instances[child_state.id].get_registered_output_keys() + self.states_instances[child_state.id].get_registered_input_keys():
                    #self.root_SM.register_io_keys(keys)
        if(len(rootInterface.data)>0):
            self.root_SM.register_start_cb(convert_datamodel_to_userdata,[rootInterface.data])
        self.root_SM.set_initial_state([rootInterface.initial_state_id], smach.UserData())
        return self.root_SM
    
    def create_all_compound_states(self):
        states = {}
        for stateInterface in self.Interface.compoundStates:
            id, state = self.create_compound_state(stateInterface)
            states[id] = state
        return states
    
    def create_compound_state(self, stateInterface):
        #PROVIDER 
        state = smach.StateMachine(stateInterface.get_outcomes())
        with state:
            for child_state in stateInterface.states:
                state.add(child_state.id, self.states_instances[child_state.id], child_state.transitions)
                ##Register IO Keys
                #for keys in self.states_instances[child_state.id].get_registered_output_keys() + self.states_instances[child_state.id].get_registered_input_keys():
                    #state.register_io_keys(keys)
                    
        state.set_initial_state([stateInterface.initial_state_id], smach.UserData())
        if(len(stateInterface.data)>0):
            state.register_start_cb(convert_datamodel_to_userdata,[stateInterface.data])  
        return stateInterface.id, state
    
    def create_all_simple_states(self):
        states = {}
        for stateInterface in self.Interface.simpleStates:
            id, state = self.create_simple_state(stateInterface)
            states[id] = state
        return states
    
    
    def get_python_state_automatch(self, stateInterface):
            ##Use Auto matched
        matching_states = []
        for provider in self.skill_provider:
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
            for provider in self.skill_provider:
                found_state = provider.get_state(stateInterface.data["python_state"])
                rospy.logdebug("State '%s' linked to '%s' python state set."%(stateInterface.id, stateInterface.data["python_state"]))
            if found_state == None:
                #No matching found for the user define state
                raise NameError("The linked Python State '%s' for the State '%s' was not found. Please check Providers.")
        else:
            found_state = self.get_python_state_automatch(stateInterface)
        
        if(found_state is not None):
            if not(isinstance(found_State, smach.State)):
                ##Not a smachState for the smach  builder
                raise TypeError("State '%s' linked Python State is not Smach State instances"%(stateInterface.id))
            else:
                return found_state
            
    
    def create_simple_state(self, stateInterface):
        #PROVIDER
        ##Use define matching
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
        