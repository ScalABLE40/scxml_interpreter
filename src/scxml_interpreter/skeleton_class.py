#!/usr/bin/env python


class SimpleStateSkeleton(object):
    def __init__(self, id, data={}, transitions={}):
        self.id = id
        self.data = data
        self.transitions = transitions
        
    def get_outcomes(self):
        return list(self.transitions.keys())
    
    def __str__(self):
        return ("SimpleStateSkeleton(\nid=%s\ndata=%s\ntransitions=%s\n)"%(str(self.id),str(self.data),str(self.transitions)))
        
class CompoundStateSkeleton(object):
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
    def __str__(self):
        state_print = ""
        for state in self.states:
            try:
                state_print =  state_print + '  ' + state.id + '\n'
            except AttributeError:
                pass
            try:
                state_print =  state_print + '  ' + state + '\n'
            except AttributeError:
                pass

        return ("CompoundStateSkeleton(\nid=%s\ndata=%s\ntransitions=%s\nstates=[\n%s]\ninitial_state_id=%s\n)"%(
            str(self.id),str(self.data),str(self.transitions),str(state_print),str(self.initial_state_id))
        )

class RootStateSkeleton(object):
    def __init__(self, initial_state_id="", data={}, final_states=[], states=[]):
        self.initial_state_id = initial_state_id
        self.data = data
        self.final_states = final_states
        self.states = states
        
    def __str__(self):
        state_print = ""
        for state in self.states:
            try:
                state_print =  state_print + '  ' + state.id + '\n'
            except AttributeError:
                pass
            try:
                state_print =  state_print + '  ' + state + '\n'
            except AttributeError:
                pass

        return ("RootStateSkeleton(\ninitial_state_id=%s\ndata=%s\nfinal_states=%s\nstates=[\n%s])"%(
            str(self.initial_state_id),str(self.data),str(self.final_states),str(state_print))
        )
        
class SCXMLSkeleton(object):
    def __init__(self):
        self.simpleStates = []
        self.compoundStates = []
        self.rootState = None
        
    def debug_print(self):
        return str(self)
        
    def __str__(self):
        str_print = ""
        for state in self.simpleStates:
            str_print = str_print + str(state) + "\n"
        for state in self.compoundStates:
            str_print = str_print + str(state) + "\n"
        str_print = str_print + str(self.rootState)
        return str_print
            
        
        
