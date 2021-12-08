#!/usr/bin/env python3

import sys
import rospy
import importlib
import smach
import inspect
import pkgutil


class SmachStateProvider(object):
    def __init__(self, package):
        self._pkg = package
        self._loaded_states = {}
        self._get_all_smach_states(package)

    def _get_all_smach_states(self, pkg_name):
        pkg = importlib.import_module(pkg_name)
        for ___, modname, ____ in pkgutil.iter_modules(pkg.__path__):
            module = pkg_name + "." + str(modname)
            if not (module in sys.modules):
                module_ref = importlib.import_module(module)
                for dir_name in dir(module_ref):
                    dir_obj = getattr(module_ref, dir_name)
                    if inspect.isclass(dir_obj):
                        if issubclass(dir_obj, smach.State):
                            rospy.logdebug("[StateProvider] Loading %s from %s" % (dir_name, str(dir_obj)))
                            if dir_name in self._loaded_states.keys():
                                new_name = str(modname) + "_" + dir_name
                                rospy.logwarn(
                                    "[StateProvider] Two states with the same name are found in the provider.\
                                \n1.%s from %s\n2.%s from %s. It will be renamed %s in the provider."
                                    % (dir_name, [dir_name], dir_name, dir_obj, new_name)
                                )
                                self._loaded_states[new_name] = dir_obj
                            else:
                                self._loaded_states[dir_name] = dir_obj

    def reload(self):
        # for state in self._loaded_states:
        # self._get_all_smach_states(self._pkg)
        pass

    def get_state(self, state_name):
        for key in self._loaded_states.keys():
            if state_name.startswith(key):
                return self._loaded_states[key]()
        return None
