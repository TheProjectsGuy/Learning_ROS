# Basic Nodes (Python)

This package contains nodes written in Python. Since ROS Noetic, the default python interpreter is Python 3.

## Table of contents

- [Basic Nodes (Python)](#basic-nodes-python)
    - [Table of contents](#table-of-contents)
    - [Files](#files)
        - [Preexisting files](#preexisting-files)
        - [Created Nodes](#created-nodes)
            - [Python Nodes](#python-nodes)
    - [Reference](#reference)

## Files

About the files in this package

### Preexisting files

When the package was created, the following files were present in the package:

1. [CMakeLists.txt](./CMakeLists.txt): This is the make file of this package. It consists the instructions on building the package (catkin tool needs them). When we create nodes written in C++, they have to be built into executables. For that to happen, the procedure has to be described explicitly. This file is primarily for that purpose.

    > Find more about it on [roswiki](http://wiki.ros.org/catkin/CMakeLists.txt)

2. [package.xml](./package.xml): Called the package manifest, this file is used to define the properties of a package. Things like the dependencies, version, maintainers, etc. You can specify the packages that this package depends upon for build and execution in this file.

    > Find more about it on [roswiki](http://wiki.ros.org/catkin/package.xml)

Along with these, there may also be some folders created. These must be empty and shall be described when the need arises.

1. `src`: This folder is to contain the source code (primarily C++). The source code for C++ nodes are written here.
2. `include`: This folder consists auxiliary files that you could `#include` in your code. More about this later.

We shall create a folder named `scripts` for storing the python code written.

### Created Nodes

All files created in this folder are to represent a node. Note that, for this package, each file represents a unique node.

#### Python Nodes

Nodes created in Python (ROS Noetic onwards uses Python 3). Python files could be placed in the same `src` folder, but it is preferred to keep them in a separate folder named `scripts`. The executable name and the script name are same, so to run a python node, use its script name as the executable name of the node.

| S. No. | Name | Notes |
| :---: | :--- | :--- |
| 1 | [basic_args_out.py](./scripts/basic_args_out.py) | Prints the arguments passed to the node |
| 2 | [basic_logging.py](./scripts/basic_logging.py) | Prints log messages of different priority levels and shows how to attach a shutdown hook |

## Reference

- `rospy`: ROS client for Python [documentation](http://wiki.ros.org/rospy)
- Installing Python nodes [reference](http://docs.ros.org/en/api/catkin/html/howto/format2/installing_python.html)
