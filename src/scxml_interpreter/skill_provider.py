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


import os
import sys
import rospy
import importlib


class SkillProvider():

    def __init__(self, package):
        self.package = package

    def load(self, module, name):
        try:
            module = self.package + "." + module
            if(module in sys.modules):
                module_ref = reload(sys.modules[module])
            else:
                module_ref = importlib.import_module(module)
        except Exception as ex:
            rospy.logerr('[SCXML Interpreter] Error in the "%s" file (pkg %s) : %s'%(module, self.package, str(ex)))
            print('[SCXML Interpreter] Error in the "%s" file (pkg %s) : %s'%(module, self.package, str(ex)))
            raise Exception(ex)
        try:
            state_instance = module_ref.__getattribute__(name)()
        except Exception as ex:
            rospy.logerr('[SCXML Interpreter] Error in the class "%s" (pkg %s, file %s) : %s'%(name,self.package, module, str(ex)))
            raise Exception(ex)
        return state_instance
