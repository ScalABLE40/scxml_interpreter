#!/usr/bin/env python

import smach
import rospy
import ast
from scxml_interpreter.interfaces import ParallelStateInterface,\
                                         CompoundStateInterface

def convert_datamodel_to_userdata(ud, initial_state, *args):
    for data_id, data_value in args[0].iteritems():
        try:
            ud[data_id] = ast.literal_eval(data_value)
        except Exception:
            ud[data_id] = data_value


def transition_convertion(ud, actives_states, *args):
    if(args[0] in actives_states):
        convert_datamodel_to_userdata(ud, "", args[1])


class SmachBuilder(object):
    def __init__(self, interface, provider):
        self._interface = interface
        self._provider = provider
        self.states_instances = {}
        self.userdata = smach.UserData()

    def build_state_machine(self):
        root_SM = smach.StateMachine(self._interface.final_states)
        self.create_child_states(self._interface, root_SM)
        root_SM.set_initial_state([self._interface.initial_state_id])
        return root_SM, self.userdata

    def create_child_states(self, parent_interface, parent_SM):
        if parent_interface.data:
            parent_SM.register_start_cb(convert_datamodel_to_userdata, [parent_interface.data])
            for data_id, _ in parent_interface.data.iteritems():
                parent_SM.register_io_keys([data_id])

        with parent_SM:
            for child_interface in parent_interface.childs_interface:
                if isinstance(child_interface, ParallelStateInterface):
                    child_state = self.create_parallel_state(child_interface)
                    self.create_child_parallel(child_interface, child_state)
                elif isinstance(child_interface, CompoundStateInterface):
                    child_state = self.create_compound_state(child_interface)
                    self.create_child_states(child_interface, child_state)
                else:
                    child_state = self.create_simple_state(child_interface)
                parent_SM.add(child_interface.identifier, child_state, child_interface.transitions)
                self._register_states_cb(parent_SM, parent_interface, child_state, child_interface, True)

        for i_key in set(parent_SM.get_registered_output_keys()).union(set(parent_SM.get_registered_input_keys())):
            self.userdata._data[i_key] = None

    def create_child_parallel(self, parent_interface, parent_SM):
        if parent_interface.data:
            parent_SM.register_start_cb(convert_datamodel_to_userdata, [parent_interface.data])
            for data_id, _ in parent_interface.data.iteritems():
                parent_SM.register_io_keys([data_id])

        with parent_SM:
            for child_interface in parent_interface.childs_interface:
                if isinstance(child_interface, ParallelStateInterface):
                    child_state = self.create_parallel_state(child_interface)
                    self.create_child_parallel(child_interface, child_state)
                elif isinstance(child_interface, CompoundStateInterface):
                    child_state = self.create_compound_state(child_interface)
                    self.create_child_states(child_interface, child_state)
                else:
                    child_state = self.create_simple_state(child_interface)
                parent_SM.add(child_interface.identifier, child_state)
                self._register_states_cb(parent_SM, parent_interface, child_state, child_interface, True)

        for i_key in set(parent_SM.get_registered_output_keys()).union(set(parent_SM.get_registered_input_keys())):
            self.userdata[i_key] = None

    def _register_states_cb(self, open_sm, sm_interface, child_state, child_interface, add_keys):
        if(child_interface.identifier == sm_interface.initial_state_id):
            open_sm.register_start_cb(convert_datamodel_to_userdata, [child_interface.data])
        else:
            open_sm.register_transition_cb(transition_convertion, [child_interface.identifier, child_interface.data])
        if(add_keys):
            for keys in set(child_state.get_registered_output_keys())\
                        .union(set(child_state.get_registered_input_keys())):
                open_sm.register_io_keys([keys])

    def create_simple_state(self, state_interface):
        return self.get_python_state(state_interface)

    def create_compound_state(self, state_interface):
        state = smach.StateMachine(state_interface.get_outcomes())
        state.set_initial_state([state_interface.initial_state_id])
        return state

    def create_parallel_state(self, state_interface):
        return smach.Concurrence(state_interface.get_outcomes(),
                                 default_outcome="failed",
                                 outcome_map=state_interface.outcome_map,
                                 )

    def get_python_state_automatch(self, stateInterface):
            ##Use Auto matched
        matching_states = []
        for provider in self._provider:
            match_state = provider.get_state(stateInterface.identifier)
            if(match_state is not None):
                matching_states.append(match_state)

        if(len(matching_states) == 0):
            ##No state found in the registered provider
            ##Raise Error Code
            raise NameError("No State matching name found for the %s state. Please check Providers."%(stateInterface.identifier))
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
                rospy.logdebug("State '%s' linked to '%s' python state set." % (stateInterface.identifier, stateInterface.data["python_state"]))
            if found_state is None:
                #No matching found for the user define state
                raise NameError("The linked Python State '%s' for the State '%s' was not found. Please check Providers."
                                % (stateInterface.id, stateInterface.data["python_state"]))
        else:
            found_state = self.get_python_state_automatch(stateInterface)

        if(found_state is not None):
            if not(isinstance(found_state, smach.State)):
                ##Not a smachState for the smach  builder
                raise TypeError("State '%s' linked Python State is not Smach State instances" % stateInterface.id)
            else:
                return found_state
