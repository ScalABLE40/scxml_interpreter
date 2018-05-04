#!/usr/bin/env python

from skill_provider import SkillProvider
import smach
import ast
from docutils.nodes import compound

def convert_datamodel_to_userdata(ud, local_ud, initial_state, args):
    for data_id,data_value in args[0]:
        ud[data_id] = ast.literal_eval(data_value) 

class SmachBuilder():
    def __init__(self, skeleton=None, skill_provider=None):
        self.skeleton = skeleton
        self.skill_provider = skill_provider
        self.root_SM = None
        self.states_instances = {}
        
    def create_root_state_machine(self):
        rootSkeleton = self.skeleton.rootState
        self.root_SM = smach.StateMachine()
        for final_state in rootSkeleton.final_states:
            self.root_SM.register_outcomes(final_state)
        with self.root_SM:
            for child_state in rootSkeleton.states:
                self.root_SM.add(child_state.id, self.states_instances[child_state.id], child_state.transitions)
        self.root_SM.register_start_cb(convert_datamodel_to_userdata,rootSkeleton.data)
        self.root_SM.set_initial_state(rootSkeleton.initial_state)
        return self.root_SM
    
    def create_all_coumpound_states(self):
        states = {}
        for stateSkeleton in self.skeleton.coumpoundStates:
            id, state = self.create_coumpound_state(stateSkeleton)
            states[id] = state
        return states
    
    def create_coumpound_state(self, stateSkeleton):
        #PROVIDER 
        state = smach.StateMachine()
        for outcome in stateSkeleton.get_outcomes():
            state.register_outcomes(outcome)
            
        with state:
            for child_state in stateSkeleton.states:
                state.add(child_state.id, self.states_instances[child_state.id], child_state.transitions)
        state.set_initial_state(stateSkeleton.initial_state_id)   
        state.register_start_cb(convert_datamodel_to_userdata,stateSkeleton.data)  
        return stateSkeleton.id, state
    
    def create_all_simple_states(self):
        states = {}
        for stateSkeleton in self.skeleton.simpleStates:
            id, state = self.create_simple_state(stateSkeleton)
            states[id] = state
        return states
    
    def create_simple_state(self, stateSkeleton):
        #PROVIDER
        state = SkillProvider.search_skill(stateSkeleton.id)
        #state = smach.State()
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
        compoundstates = self.create_all_coumpound_states()
        states_instances = compoundstates.copy()
        self.states_instances.update(states_instances)
        return self.create_root_state_machine()
        
    
            
            
        





















