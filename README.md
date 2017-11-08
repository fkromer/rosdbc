# rosdbc

`rosdbc` is a ROS meta-package containing utility functionality for the main ROS
client libraries (Python, C++) which eases the emulation of "Design By Contract"
on the ROS node/nodelet level.

If you already know about the concept of "Design by Contract" skip the
introductory section and read section [Usage](#usage) to learn how to use
`rosdbc` in your ROS nodes/nodelets.

  * [Introduction](#introduction)
    * [Design by Contract (DbC)](#design-by-contract-dbc)
    * [An executable function level contract example](#an-executable-function-level-contract-example)
    * [DbC in ROS](#dbc-in-ros)
    * [Benefits when using DbC](#benefits-when-using-dbc)
  * [Installation](#installation)
  * [Usage](#usage)

## Introduction

If the concept of "Design By Contract" is new to you keep going reading this
section to get some basic understanding of it. Otherwise skip to the sections
[Installation](#installation) and section [Usage](#usage) to learn how to
installation and use `rosdbc` in your ROS nodes/nodelets.

### Design by Contract (DbC)

The two following citations describe this concept fairly well.

"Contract programming (note: Design By Contract) is a software design approach
that treats parts of software as individual entities that provide services to
each other. This approach realizes that software can work according to its
specification as long as the provider and the consumer of the service both obey
a contract."
[online version of "Programming in D - Tutorial and reference", chapter "Contract Programming"](http://ddili.org/ders/d.en/contracts.html)

“It (Design By Contract) prescribes that software designers should define formal,
precise and verifiable interface specifications for software components, which
extend the ordinary definition of abstract data types with preconditions,
postconditions and invariants."
[wikipedia.org- Design By contract](https://en.wikipedia.org/wiki/Design_by_contract)

Preconditions must be ensured by (a) the client software of a software
component and/or (b) the system the software component is run in.

Invariants (valid/consistent state in the parent context) must be ensured by
(a) the software component and/or (b) the system the software component
is run in.

Postconditions must be ensured by (a) the software component and/or (b) the system
the software is running in.

Consider a (member) function as software component. Every code calling the function is
client software. The calling codes context, any other higher level entity of the
function or even the operating system in which the function is executed in can
be the parent context. If the client code calls the function with invalid
arguments it violates the functions "precondition" contract which is required
that it can ensure its proper functioning. If a programming error in the
function corrupts the parent context (e.g. changing data in, very bad, global
variables) the "invariant" contract would be violated. If a programming error
would change the member functions class instance data unintentional this would
violate the "invariant" contract. If the function is consuming too much memory
and the operating systems is not capable of providing enough memory would violate
the "invariant" as well as the "out" contract.

### An executable function level example

To understand how DbC may look like in practice in the library function level
(the function is considered as the "software component" here) go on reading this
section.

Because DbC in the programming language [D](https://dlang.org/) is the most
familiar to me I use an
[function level contract example written in D](http://ddili.org/ders/d.en/contracts.html).

    import std.stdio;

    /* Distributes the sum between two variables.
    *
    * Distributes to the first variable first, but never gives
    * more than 7 to it. The rest of the sum is distributed to
    * the second variable. */
    void distribute(in int sum, out int first, out int second)
    in {
        assert(sum >= 0);

    } out {
        assert(sum == (first + second));

    } body {
        first = (sum >= 7) ? 7 : sum;
        // this programming error would violate the out assert and
        // raise core.exception.AssertError which is an error, no exception
        //second = sum + first;
        second = sum - first;
    }

    unittest {
        int first;
        int second;

        // Both must be 0 if the sum is 0
        distribute(0, first, second);
        assert(first == 0);
        assert(second == 0);

        // If the sum is less than 7, then all of it must be given
        // to first
        distribute(3, first, second);
        assert(first == 3);
        assert(second == 0);

        // Testing a boundary condition
        distribute(7, first, second);
        assert(first == 7);
        assert(second == 0);

        // If the sum is more than 7, then the first must get 7
        // and the rest must be given to second
        distribute(8, first, second);
        assert(first == 7);
        assert(second == 1);

        // A random large value
        distribute(1_000_007, first, second);
        assert(first == 7);
        assert(second == 1_000_000);
    }

    void main() {
        int first;
        int second;

        // this usage error inside the system would violate the "in" assert
        // and raise a core.exception.AssertError which is an error, no exception
        // (if the function would be part of an API means not for usage inside
        // the system an "enforce" which raises an exception in the function
        // body would be more suitable)
        //distribute(-1, first, second);
        distribute(123, first, second);
        writeln("first: ", first, " second: ", second);
    }

In the examples context the function `void distribute(in int sum, out int first, out int second)`
is the software component. The function `main()` invoked when the file is run as
script is the client software. If the programmer of the function would introduce
a bug like (`//second = sum + first;` instead of `second = sum - first;`) he
would violate the contracts postcondition
"out" block //second = sum + first;


If you want to execute the example you can install D (DMD compiler, runtime
library, etc.) on Ubuntu and activate the D environment with

    curl -fsS https://dlang.org/install.sh | bash -s dmd
    source ~/dlang/dmd-2.074.1/activate

You can execute the example above by copy & pasting the code into a file
`contract.d` and by executing it with

    rdmd contract.d

Finally the D environment needs to be deactivate with

    deactivate

## DbC in ROS

TODO

### Benefits when using DbC

TODO

## Installation

### Python package installation

    cd <catkin-workspace>/src
    git clone https://github.com/fkromer/rosdbc.git
    cd ..
    catkin_make --pkg rosdbcpy
    . devel/setup.bash

### C++ package installation

C++ PACKAGE NOT EXISTING YET

## Usage

The packages of the meta-package are used in "library mode". This means the
package functionality will be imported as library into your own packages which
implement ROS nodes.

### Python package usage

A minimal working example. Start the `roscore` in a terminal.

    roscore

Open a new terminal to try the package functionality (here: assert if parameter
is not existing). As the parameter is not existing yet an exception is raised.

    python
    >>> import rosdbcpy
    >>> rosdbcpy.assert_parameter_not_existing("blub")
        Traceback (most recent call last):
        File "<stdin>", line 1, in <module>
        File "<string>", line 40, in assert_parameter_not_existing
        rosdbcpy.ParameterContractViolation: Parameter blub not existing
    >>>

Open a new terminal and create the parameter.

    rosparam set blub 0

Check for the parameter existence in the "rosdbcpy" terminal. As the parameter
is existing now no exception is raised.

    >>> rosdbcpy.assert_parameter_not_existing("blub")
    >>>

### C++ package Usage

C++ PACKAGE NOT EXISTING YET

## Development

### Executing all Python package tests

    cd <catkin-workspace>
    catkin_make run_tests_rosdbcpy

### Executing the Python package library level tests

install `python-nose`

    sudo apt-get install python-nose

run the tests (standalone)

    cd <catkin-workspace>
    catkin_make run_tests_rosdbcpy_nosetests

### Get summary of Python package test results

    catkin_test_results build/test_results/rosdbcpy/
