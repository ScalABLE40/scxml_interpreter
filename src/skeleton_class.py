#!/usr/bin/env python

class SimpleStateSkeleton(object):
    def __init__(self, id, data={}, transitions={}):
        self.id = id
        self.data = data
        self.transitions = transitions
        
    def get_outcomes(self):
        return list(self.transitions.keys())
        
class CoumpoundStateSkeleton(object):
    def __init__(self, id, data={}, transitions={}, states=[], initial_state_id=""):
        self.id = id
        self.data = data
        self.transitions = transitions
        self.states = states
        self.initial_state_id = initial_state_id
        
    def get_outcomes(self):
        return list(self.transitions.keys())
    
    def get_outcome_target(self, outcome):
        if(outcome in self.transitions):
            return self.transitions[outcome]
        else:
            return None ##TODO return proper error code

class RootStateSkeleton(object):
    def __init__(self, initial_state_id="", data={}, final_states=[], states=[]):
        self.initial_state = initial_state
        self.data = data
        self.final_states = final_states
        self.states = states
        
class SCXMLSkeleton(object):
    def __init__(self):
        self.simpleStates = []
        self.coumpoundStates = []
        self.rootState = None
        