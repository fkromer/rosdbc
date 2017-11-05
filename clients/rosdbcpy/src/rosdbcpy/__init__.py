#!/usr/bin/env python
# Software License Agreement (Apache License 2.0)
#
# Copyright 2017 Florian Kromer
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy

class ParameterContractViolation(Exception):
    """Basic exception for contract violations raised by parameters."""
    def __init__(self, name, msg=None):
        if msg is None:
            # default error message
            msg = "Contract violation of parameter %s" % name
        super(ParameterContractViolation, self).__init__(msg)
        # make name accessible for exception handling
        self.name = name

class ParameterValueViolation(ParameterContractViolation):
    """Exception for value contract violations raised by parameters."""
    def __init__(self, name, value):
        super(ParameterValueViolation, self).__init__(
            name, msg="Parameter %s violated contract with value %s" % (name, value))
        self.value = value

def assert_parameter_not_existing(name):
    """Assert if the parameter is not existing."""
    if not rospy.has_param(name):
        raise ParameterContractViolation(name, "Parameter %s not existing" % (name))

def assert_parameter_out_of_range(name, lower_bound, upper_bound):
    """
    Assert if a parameter is not within the valid range.
    """
    if rospy.has_param(name):
        # get parameter values
        if lower_bound > parameter > upper_bound:
            raise ParameterValueViolation(name, value)
    else:
        raise ParameterContractViolation("Parameter not existing.")
