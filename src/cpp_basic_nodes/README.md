# Basic Nodes (C++)

This package contains basic nodes for understanding core concepts of ROS. Since this is the first package, things are explained in more detail.

## Table of contents

- [Basic Nodes (C++)](#basic-nodes-c)
    - [Table of contents](#table-of-contents)
    - [Files](#files)
        - [Preexisting files](#preexisting-files)
        - [Created Nodes](#created-nodes)
            - [C++ Nodes](#c-nodes)
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

### Created Nodes

All files created in this folder are to represent a node. Note that, for this package, each file represents a unique node.

#### C++ Nodes

Nodes created in C++. It is suggested that you read the entire files (as comments have material)

| S. No. | Name | Node name | Notes |
| :---: | :--- | :--- | :---- |
| 1 | [basic_args_out.cpp](./src/basic_args_out.cpp) | cpp_basic_args_out | Prints the arguments passed to the node |
| 2 | [basic_logging.cpp](./src/basic_logging.cpp) | | Prints log messages of different priority levels and also shows how to create a custom SIGINT handler |

## Reference

- `roscpp` [API documentation](https://docs.ros.org/en/api/roscpp/html/)
- `rosconsole` Logging [documentation](http://wiki.ros.org/rosconsole)
