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
    """
    Basic exception for contract violations raised by parameters.
    """
    def __init__(self, name, msg=None):
        if msg is None:
            # default error message
            msg = "Contract violation of parameter %s" % name
        super(ParameterContractViolation, self).__init__(msg)
        # make name accessible for exception handling
        self.name = name

class ParameterValueViolation(ParameterContractViolation):
    """
    Exception for value contract violations raised by parameters.
    """
    def __init__(self, name, value):
        super(ParameterValueViolation, self).__init__(
            name, msg="Parameter %s violated contract with value %s" % (name, value))
        self.value = value

def assert_parameter_exists(name):
    """
    Indicates a contract violation if the parameter is expected to exist but if
    it does not exist.

    Args:
        name (string): Name of the parameter.

    Raises:
        ParameterContractViolation: Raised if parameter is not existing.
    """
    if not rospy.has_param(name):
        raise ParameterContractViolation(name, "Parameter %s not existing" % (name))

def assert_parameter_not_exists(name):
    """
    Indicates a contract violation if the parameter is expected to not exist but
    if it does exist.

    Args:
        name (string): Name of the parameter.

    Raises:
        ParameterContractViolation: Raised if parameter is existing.
    """
    if rospy.has_param(name):
        raise ParameterContractViolation(name, "Parameter %s existing" % (name))

def assert_parameter_has_value(name, value):
    """
    Indicates a contract violation if it is expected that the parameter has a
    specific value but if it has not.

    Args:
        name (string): Name of the parameter.
        value (depends on the parameter type): Value of the parameter.

    Raises:
        ParameterValueViolation: Raised if parameter value is not like expected.
    """
    if rospy.has_param(name):
        observed_value = rospy.get_param(name)
        if value != observed_value:
            ParameterValueViolation(name, value)
    else:
        raise ParameterContractViolation(name, "Parameter %s not existing" % (name))

def assert_parameter_in_range(name, lower_bound, upper_bound):
    """
    Indicates a contract violation if it is expected that a parameter value of
    type 32-bit integers has a value within a defined range but if it has not.

    Args:
        name (string): Name of the parameter.

    Raises:
        ParameterValueViolation: Raised if parameter value is not in the range.
        ParameterContractViolation: Raised if parameter does not exist.
    """
    if rospy.has_param(name):
        value = rospy.get_param(name)
        if lower_bound > value > upper_bound:
            raise ParameterValueViolation(name, value)
    else:
        raise ParameterContractViolation(name, "Parameter %s not existing" % (name))

def assert_parameter_out_range(name, lower_bound, upper_bound):
    """
    Indicates a contract violation if it is expected that a parameter value of
    type 32-bit integers has a value outside a defined range but if it has not.

    Args:
        name (string): Name of the parameter.

    Raises:
        ParameterValueViolation: Raised if parameter value is not outside the range.
        ParameterContractViolation: Raised if parameter does not exist.
    """
    if rospy.has_param(name):
        value = rospy.get_param(name)
        if lower_bound > value > upper_bound:
            raise ParameterValueViolation(name, value)
    else:
        raise ParameterContractViolation(name, "Parameter %s not existing" % (name))
