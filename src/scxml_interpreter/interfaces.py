#!/usr/bin/env python
class SimpleStateInterface(object):
    def __init__(self, id, data={}, transitions={}, onentry={}, onexit={}):
        self.id = id
        self.data = data
        self.transitions = transitions
        self.onentry = onentry
        self.onexit = onexit

    def get_outcomes(self):
        return list(self.transitions.keys())

    def __str__(self):
        return ("SimpleStateInterface(\nid=%s\ndata=%s\ntransitions=%s\nonentry=%s\nonexit=%s\n)"%(str(self.id),str(self.data),str(self.transitions),str(self.onentry),str(self.onexit)))


class ContainerStateInterface(SimpleStateInterface):
    def __init__(self, id, data={}, transitions={}, states=[], onentry={}, onexit={}):
        SimpleStateInterface.__init__(self, id, data, transitions, onentry, onexit)
        self.childs_interface = states

    def get_outcome_target(self, outcome):
        if(outcome in self.transitions):
            return self.transitions[outcome]
        else:
            return None ##TODO return proper error code


class CompoundStateInterface(ContainerStateInterface):
    def __init__(self, id, data={}, transitions={}, states=[], initial_state_id="", onentry={}, onexit={}):
        ContainerStateInterface.__init__(self, id, data, transitions, states, onentry, onexit)
        self.initial_state_id = initial_state_id

    def __str__(self):
        state_print = ""
        for state in self.childs_interface:
            if(isinstance(state, SimpleStateInterface) or isinstance(state, CompoundStateInterface)):
                state_print = state_print + '  ' + state.id + '\n'
            else:
                state_print = state_print + '  ' + state + '\n'

        return ("CompoundStateInterface(\nid=%s\ndata=%s\ntransitions=%s\nstates=[\n%s]\ninitial_state_id=%s\nonentry=%s\nonexit=%s\n)"%(
            str(self.id),str(self.data),str(self.transitions),str(state_print),str(self.initial_state_id),str(self.onentry),str(self.onexit))
        )


class ParallelStateInterface(ContainerStateInterface):
    def __init__(self, id, data={}, default_outcome=None, outcome_map=[], transitions={}, states=[], onentry=[], onexit=[]):
        ContainerStateInterface.__init__(self, id, data, transitions, states, onentry, onexit)
        self.default_outcome = default_outcome
        self.outcome_map = outcome_map
        self.initial_state_id = "NO INITIAL"

    def __str__(self):
        state_print = ""
        for state in self.childs_interface:
            try:
                state_print = state_print + '  ' + state.id + '\n'
            except AttributeError:
                pass
            try:
                state_print = state_print + '  ' + state + '\n'
            except AttributeError:
                pass

        return ("ParallelStateInterface(\nid=%s\ndata=%s\ntransitions=%s\nstates=[\n%s]\ninitial_state_id=%s\nonentry=%s\nonexit=%s\n)"%(
            str(self.id),str(self.data),str(self.transitions),str(state_print),str(self.initial_state_id),str(self.onentry),str(self.onexit))
        )


class SCXMLInterface(CompoundStateInterface):
    def __init__(self, data={}, final_states=[], states=[], initial_state_id=""):
        CompoundStateInterface.__init__(self, "", data, {}, states, initial_state_id)
        self.final_states = final_states

    def __str__(self):
        state_print = ""
        for state in self.childs_interface:
            state_print = state_print + '  ' + state.id + '\n'

        return ("RootStateInterface(\ninitial_state_id=%s\ndata=%s\nfinal_states=%s\nstates=[\n%s])"
                % (str(self.initial_state_id), str(self.data), str(self.final_states), str(state_print)))
