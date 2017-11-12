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

import sys
import unittest

import rosunit
import rosdbcpy
import rospy
import subprocess

class ParameterContractViolationExceptions(unittest.TestCase):
    """Tests for the parameter contract violation exceptions."""

    def setUp(self):
        """Test class level setup executed before every test. Start the roscore."""
        self.roscore = subprocess.Popen("roscore")

    def tearDown(self):
        """Test class level teardown executed after every test. Stop the roscore."""
        self.roscore.terminate()

    def test_non_existing_parameters_raises_exception(self):
        """Verifies if an exception is raised due to a parameter which is
        expected to exist."""
        with self.assertRaises(rosdbcpy.ParameterContractViolation):
            rosdbcpy.assert_parameter_exists("blub")

    def test_existing_parameter_raises_exception(self):
        """Verifies if an exception is raised due to a parameter which is not
        expected to exist."""
        # specific function level setup
        irrelevant_value = 1
        rospy.set_param('/blub', irrelevant_value)

        try:
            # execution
            with self.assertRaises(rosdbcpy.ParameterContractViolation):
                rosdbcpy.assert_parameter_not_exists("blub")
        finally:
            # specifc function level teardown
            rospy.delete_param('/blub')

if __name__ == '__main__':
    rosunit.unitrun('test_parameter_checks', 'test_non_existing_parameters_raises_exception', ParameterContractViolationExceptions, coverage_packages=['rosdbcpy'])
