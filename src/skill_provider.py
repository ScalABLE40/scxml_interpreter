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

from xml.etree import ElementTree
import importlib


class SkillProvider(object):

    def __init__(self, skill_register_file):

        if not os.path.isfile(skill_register_file):
            raise Exception("Invalid register file path !")

        self._tree = ElementTree.parse(skill_register_file)
        self._root = self._tree.getroot()

    def load(self, name):

        """ Find skill registered by name
        <skill name="${name}" pkg="${pkg}" module="${module}" class="${class}"/>
        """

        skill = self._root.find('skill[@name="%s"]'%name)
        if skill is not None:
            try:
                in_pkg          = skill.attrib["pkg"]
                in_module       = skill.attrib["module"]
                skill_class     = skill.attrib["class"]
            except Exception as ex:
                rospy.logerr('[SCXML Interpreter] Error in the file register : %s'%(str(ex)))
                raise Exception(ex)
            skill_class_ref = None
            try:
                if(in_module in sys.modules):
                    module_ref = reload(sys.modules[in_module])
                else:
                    module_ref = importlib.import_module(in_module, in_pkg)
            except Exception as ex:
                rospy.logerr('[SCXML Interpreter] Error in the "%s" file (pkg %s) : %s'%(in_module, in_pkg, str(ex)))
                raise Exception(ex)
            try:
                skill_class_ref = module_ref.__getattribute__(skill_class)
            except Exception as ex:
                rospy.logerr('[SCXML Interpreter] Error in the class "%s" (pkg %s, file %s) : %s'%(skill_class,in_pkg,in_module, str(ex)))
                raise Exception(ex)
            return skill_class_ref
        else:
            rospy.logerr('[SCXML Interpreter] Skill named "%s" not found in register file !'%name)
            return None
