# Basic Nodes (Python)

Basic Python Nodes to understand essential concepts and the build procedure for a python package.

## Table of contents

- [Basic Nodes (Python)](#basic-nodes-python)
    - [Table of contents](#table-of-contents)
    - [Creating this package](#creating-this-package)
    - [Foreword](#foreword)
    - [Nodes](#nodes)
        - [Simple Hello World](#simple-hello-world)
            - [Creating executable](#creating-executable)
            - [Running a python node](#running-a-python-node)

## Creating this package

This package was creates using the following commands

```bash
cd ~/ros_workspaces/learning_ws/src
catkin_create_pkg py_basic_nodes rospy
```

## Foreword

This is teh first package and hence things are described in a little more detail here. The source code has comments describing the contents that are new.

Make sure you know how to build a package before proceeding.

## Nodes

Nodes declared in this package

### Simple Hello World

| Field | Value |
| :--- | :---- |
| Node name | `simple_hello_world_py` |
| Code | [scripts/simple_hello_world.py](./scripts/simple_hello_world.py) |

Node prints `Hello, World!` and different levels of logging messages.

#### Creating executable

To create an executable, add the following to the `CMakeLists.txt` file

1. Go to the `Install` section (it must be decorated with "Install" heading)
    1. Scroll to the `catkin_install_python` function

        Add the following line immediately after the comment block

        ```txt
        catkin_install_python(PROGRAMS
            scripts/simple_hello_world.py
            DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
        ```

        This is necessary to create an executable file for your python script.

After that, build the package by running `catkin_make` in the workspace directory. After a successful build process, you must see and executable named `simple_hello_world.py` in the directory `devel/lib/py_basic_nodes` (inside the workspace). This means that the workspace stores all the executables in the `devel` folder.

A similar procedure shall be followed for other nodes, so only the function names shall be mentioned hereon.

#### Running a python node

First, make sure that `roscore` is running. To run this node, run

```bash
rosrun py_basic_nodes simple_hello_world.py
```

You can even locate this package by running

```bash
rospack find py_basic_nodes
```

After inspecting the output of the following

```bash
rosnode list
```

One can say that the name of the node is what's mentioned in the `rospy.init_node` function.

Hereon, only the `rosrun` command (the bare minimum) shall be described.

After that, try running the node as

```bash
rosrun py_basic_nodes simple_hello_world.py arg1 arg2 arg3
```

You may kill the node using

```bash
rosnode kill /hello_world_simple_py
```

After running the node, a few observations can be made:

- You have successfully run your first Python ROS node.
- The first argument passed to any executable is the full path of the executable, followed by arguments passed during the call.
- Proper logging etiquette is observed. Do not use `print` to log things.
