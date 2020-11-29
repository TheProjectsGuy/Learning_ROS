# Basic Nodes (C++)

Basic C++ Nodes to understand essential concepts and the build procedure for a C++ package.

> Link to the README of the `src` folder [here](../README.md)

## Table of contents

- [Basic Nodes (C++)](#basic-nodes-c)
    - [Table of contents](#table-of-contents)
    - [Creating this package](#creating-this-package)
    - [Foreword](#foreword)
    - [Nodes](#nodes)
        - [Simple Hello World](#simple-hello-world)
            - [Building a C++ Node](#building-a-c-node)
            - [Running the Node](#running-the-node)
        - [Simple Publisher](#simple-publisher)
    - [Reference](#reference)

## Creating this package

This package was created using the following commands

```bash
cd ~/ros_workspaces/learning_ws/src
catkin_create_pkg cpp_basic_nodes roscpp
```

## Foreword

This is the first package and hence things are described in a little more detail here. The source code has comments describing the contents that are new.

Make sure you know how to build a package before proceeding.

## Nodes

Nodes declared in this package

### Simple Hello World

| Field | Value |
| :-- | :--- |
| Node name | `simple_hello_world` |
| Code | [src/simple_hello_world.cpp](./src/simple_hello_world.cpp) |

Node prints `Hello, World!` and different levels of logging messages.

#### Building a C++ Node

To build the node, add the following in the `CMakeLists.txt` file in the package

1. Go to the `Build` section (it must be decorated with "Build" heading)
    1. Scroll to the `add_executable` function

        Add the following lines immediately after the comment block

        ```txt
        add_executable(simple_hello_world src/simple_hello_world.cpp)
        ```

        This will create an executable named `simple_hello_world` from the source code `simple_hello_world.cpp`.

        > More on `add_executable` in `CMakeLists.txt` [here](http://wiki.ros.org/catkin/CMakeLists.txt#Executable_Targets)
    2. Scroll to the `target_link_libraries` function

        Add the ROS libraries in this function. The executable may need function calls from the default libraries (it depends on these libraries). You can use the `${catkin_LIBRARIES}` variable to get the default libraries.

        ```txt
        target_link_libraries(simple_hello_world ${catkin_LIBRARIES})
        ```

        > More on `target_link_libraries` in `CMakeLists.txt` [here](http://wiki.ros.org/catkin/CMakeLists.txt#target_link_libraries)

After that, build the package by running `catkin_make` in the workspace directory. After a successful build process, you must see an executable named `simple_hello_world` in the directory `devel/lib/cpp_basic_nodes` (inside the workspace). This means that the workspace stores all the executables in the `devel` folder.

A similar procedure shall be followed for other nodes, so only the function names shall be mentioned hereon.

#### Running the Node

First, make sure that `roscore` is running and that the package has been built successfully and can be found using `rospack find`

```bash
rospack find cpp_basic_nodes
```

Should return the path to the package. To run the node, run

```bash
rosrun cpp_basic_nodes simple_hello_world
```

This should run the node. Inspect the output of the following

```bash
rosnode list
```

This shall show the node as `/hello_world_simple`, that is the name used in `ros::init` function in the source code. This means that the name of the node during runtime can be different from the name of the executable.

Hereon, only the `rosrun` command (the bare minimum) shall be described.

After that, try running the node as

```bash
rosrun cpp_basic_nodes simple_hello_world arg1 arg2 arg3
```

You may kill the node using

```bash
rosnode kill /hello_world_simple
```

After running the node, a few observations can be made:

- The debug messages do not appear, but information level and above messages do appear.
- Proper logging etiquette is observed. Do not use `cout` to log things.
- The first argument passed to any executable is the full path of the executable, followed by arguments passed during the call.
- The second debug message was visible (changed logger level in code).
- You have successfully run your first C++ ROS node.

### Simple Publisher

| Field | Value |
| :--- | :--- |
| Node name | `simple_cpp_publisher` |
| Code | [src/simple_publisher.cpp](./src/simple_publisher.cpp) |

Node publishes a message on a topic named `/simple_cpp_publisher/hello_str`. Demonstrates publishing messages on a topic and name scoping.

**Building**

In the `CMakeLists.txt`, add the following

```txt
add_executable(simple_cpp_publisher src/simple_publisher.cpp)
```

and

```txt
target_link_libraries(simple_cpp_publisher ${catkin_LIBRARIES})
```

Then, run `catkin_make` in the workspace folder.

**Running**

To run the node, first run `roscore`, then

```bash
rosrun cpp_basic_nodes simple_cpp_publisher
```

Now, run

```bash
rostopic list
```

You must see `/simple_cpp_publisher/hello_str` in the output. This is because the node handle was passed the `~` in the source code (namespace has become local to the node). If you rebuild the node after removing `~`, the name would then become `/hello_str` (that is, use `ros::NodeHandle nh;` instead of `ros::NodeHandle nh("~");`).

You can inspect the contents of the messages being published by running

```bash
rostopic echo /simple_cpp_publisher/hello_str
```

This would echo messages from the point where the command was called. You are encouraged to experiment and understand things before proceeding further (same is true for everything hereon).

Kill the nodes using `rosnode kill` commands.

## Reference

- roswiki
    - [Remapping Arguments](http://wiki.ros.org/Remapping%20Arguments)
