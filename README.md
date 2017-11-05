# rosdbc
Utilities to emulate "Design By Contract" in ROS 1 (Robot Operating System).

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
