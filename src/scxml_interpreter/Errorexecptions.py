#!/usr/bin/env python
import os,sys
import rospkg
import os
#from scxml_interpreter.scxml_parser import SCXMLParser
class Error(Exception):
    """Base class for exceptions in this module."""
    pass

class ParserError(Error):
    """Exception raised for errors in the input."""

    def __init__(self, file):
        self.file = file


class InterpreterError(Error):

    def __init__(self,node_id=None,initial_state=None,event=None,target=None,condition=None,map=None,data=None,final_states=None ):
        self.node_id = node_id
        self.initial_state = initial_state
        self.event = event
        self.target = target
        self.condition = condition
        self.map = map
        self.data = data
        self.final_states = final_states
