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

class ParameterContractViolationExceptions(unittest.TestCase):
    """Tests for the parameter contract violation exceptions."""

    def test_non_existing_parameters_raises_exception(self):
        # implicit setup: no roscore and no parameter existing
        with self.assertRaises(rosdbcpy.ParameterContractViolation):
            rosdbcpy.assert_parameter_not_existing("blub")
