#!/usr/bin/env python

from skill_provider import SkillProvider
import smach
import ast

def convert_datamodel_to_userdata(ud, initial_state, *args):
    for data_id,data_value in args[0].iteritems():
        ud[data_id] = ast.literal_eval(data_value) 

class SmachBuilder():
    def __init__(self, skeleton=None, skill_provider=None):
        self.skeleton = skeleton
        self.skill_provider = skill_provider
        self.root_SM = None
        self.states_instances = {}
        
    def create_root_state_machine(self):
        rootSkeleton = self.skeleton.rootState
        self.root_SM = smach.StateMachine(rootSkeleton.final_states)
        with self.root_SM:
            for child_state in rootSkeleton.states:
                self.root_SM.add(child_state.id, self.states_instances[child_state.id], child_state.transitions)
                #for keys in self.states_instances[child_state.id].get_registered_output_keys() + self.states_instances[child_state.id].get_registered_input_keys():
                    #self.root_SM.register_io_keys(keys)
        if(len(rootSkeleton.data)>0):
            self.root_SM.register_start_cb(convert_datamodel_to_userdata,[rootSkeleton.data])
        self.root_SM.set_initial_state([rootSkeleton.initial_state_id], smach.UserData())
        return self.root_SM
    
    def create_all_compound_states(self):
        states = {}
        for stateSkeleton in self.skeleton.compoundStates:
            id, state = self.create_compound_state(stateSkeleton)
            states[id] = state
        return states
    
    def create_compound_state(self, stateSkeleton):
        #PROVIDER 
        state = smach.StateMachine(stateSkeleton.get_outcomes())
        with state:
            for child_state in stateSkeleton.states:
                state.add(child_state.id, self.states_instances[child_state.id], child_state.transitions)
                ##Register IO Keys
                #for keys in self.states_instances[child_state.id].get_registered_output_keys() + self.states_instances[child_state.id].get_registered_input_keys():
                    #state.register_io_keys(keys)
                    
        state.set_initial_state([stateSkeleton.initial_state_id], smach.UserData())
        if(len(stateSkeleton.data)>0):
            state.register_start_cb(convert_datamodel_to_userdata,[stateSkeleton.data])  
        return stateSkeleton.id, state
    
    def create_all_simple_states(self):
        states = {}
        for stateSkeleton in self.skeleton.simpleStates:
            id, state = self.create_simple_state(stateSkeleton)
            states[id] = state
        return states
    
    def create_simple_state(self, stateSkeleton):
        #PROVIDER
        state = self.skill_provider.load("WaitSkill",stateSkeleton.id)
        outcomes = list(state.get_registered_outcomes())
        for outcome in outcomes:
            if not(outcome in stateSkeleton.get_outcomes()):
                ##Raise error
                pass
        for outcome in stateSkeleton.get_outcomes():
            if not(outcome in outcomes):
                ##Raise error
                pass
        return stateSkeleton.id, state
    
    def build_state_machine(self):
        
        
        simplestates = self.create_all_simple_states()
        self.states_instances = simplestates
        compoundstates = self.create_all_compound_states()
        states_instances = compoundstates.copy()
        self.states_instances.update(states_instances)
        return self.create_root_state_machine()
        