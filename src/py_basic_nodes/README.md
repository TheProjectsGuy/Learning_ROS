# Basic Nodes (Python)

Basic Python Nodes to understand essential concepts and the build procedure for a python package.

> Link to the README of the `src` folder [here](../README.md)

## Table of contents

- [Basic Nodes (Python)](#basic-nodes-python)
    - [Table of contents](#table-of-contents)
    - [Creating this package](#creating-this-package)
    - [Foreword](#foreword)
    - [Contents](#contents)
    - [Nodes](#nodes)
        - [Simple Hello World](#simple-hello-world)
            - [Creating executable](#creating-executable)
            - [Running a python node](#running-a-python-node)
        - [Simple Publisher](#simple-publisher)
            - [Building](#building)
            - [Running](#running)
        - [Simple Subscriber](#simple-subscriber)
            - [Building](#building-1)
            - [Running](#running-1)
        - [Simple Service Server](#simple-service-server)
            - [Building](#building-2)
            - [Running](#running-2)
        - [Simple Service Client](#simple-service-client)
            - [Building](#building-3)
            - [Running](#running-3)
    - [Services](#services)
        - [AddAllFloat64Numbers_py](#addallfloat64numbers_py)
            - [Building services and messages](#building-services-and-messages)
                - [Package.xml](#packagexml)
                - [CMakeLists.txt](#cmakeliststxt)
    - [Reference](#reference)

## Creating this package

This package was creates using the following commands

```bash
cd ~/ros_workspaces/learning_ws/src
catkin_create_pkg py_basic_nodes rospy
```

## Foreword

This is teh first package and hence things are described in a little more detail here. The source code has comments describing the contents that are new.

When reading this file, you may either traverse from top to bottom (recommended for beginners) or navigate through [table of contents](#table-of-contents). If you are a beginner, you're suggested to navigate the package using the [contents](#contents) section. In the [Nodes](#nodes) section, the nodes are described starting with a table (to lead you to the source code). Same is done for other sections hereon.

Make sure you know how to build a package before proceeding.

## Contents

Suggested order of traversal for the items in this package (specially for beginners)

| S. No. | Name | Link | Description |
| :---- | :---: | :--- | :---- |
| 1 | Hello World | [Nodes > Simple Hello World](#simple-hello-world) | Prints `Hello, World!` |
| 2 | Publisher (Simple) | [Nodes > Simple Publisher](#simple-publisher) | Publish messages |
| 3 | Subscriber (Simple) | [Nodes > Simple Subscriber](#simple-subscriber) | Subscribe to messages |
| 4 | Creating AddAllFloat64Numbers_py service | [Services > AddAllFloat64Numbers_py](#addallfloat64numbers_py) | Creating and building your own service (.srv file) |
| 5 | Service Server (Simple) | [Nodes > Simple Service Server](#simple-service-server) | Server for the service AddAllFloat64Numbers_py |
| 6 | Service Client (Simple) | [Nodes > Simple Service Client](#simple-service-client) | Client for the service AddAllFloat64Numbers_py |

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

        > More about the `catkin_install_python` function in `CMakeLists.txt` [here](http://wiki.ros.org/catkin/CMakeLists.txt#Installing_Python_Executable_Scripts) and in [catkin manual](http://docs.ros.org/en/api/catkin/html/howto/format2/installing_python.html)

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

### Simple Publisher

| Field | Value |
| :--- | :---- |
| Node name | `simple_publisher.py` |
| Code | [scripts/simple_publisher.py](./scripts/simple_publisher.py) |

Node publishes a message on a topic named `/simple_py_publisher/hello_str`. Demonstrates publishing messages on a topic, name scoping and rate control.

#### Building

In the `CMakeLists.txt`, add the following to the `catkin_install_python` function

```bash
    scripts/simple_publisher.py
```

before the `DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}`. Then run `catkin_make` in the workspace folder.

#### Running

To run the node, run `roscore` first, then

```bash
rosrun py_basic_nodes simple_publisher.py
```

Now, run

```bash
rostopic list
```

You must see `/simple_py_publisher/hello_str` in the list. This is because the node was named as such in the `Publisher` initialization call. If you change `"{0}/hello_str".format(rospy.get_name())` to `hello_str`, the output would have `/hello_str` instead.

You can inspect the contents of the messages being published by running

```bash
rostopic echo /simple_py_publisher/hello_str
```

This would echo messages from the point where the command was called. You are encouraged to experiment and understand things before proceeding further (same is true for everything hereon).

Kill the nodes using `rosnode kill` commands.

### Simple Subscriber

| Field | Value |
| :--- | :---- |
| Node name | `simple_py_subscriber` |
| Code | [scripts/simple_subscriber.py](./scripts/simple_subscriber.py) |

Node subscribes to topic named `/simple_py_subscriber/subs_str`. Demonstrates subscribing to messages received on a topic.

#### Building

In the `CMakeLists.txt`, add the following in the `catkin_install_python` function

```txt
    scripts/simple_subscriber.py
```

before the `DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}`. Then run `catkin_make` in the workspace folder.

#### Running

To run the node, first run `roscore`, then

```bash
rosrun py_basic_nodes simple_subscriber.py
```

Now, after running

```bash
rostopic list
```

You must see a topic named `/simple_py_subscriber/subs_str` has been created. Get more information about it using `rostopic info`.

To publish a message on that topic at a particular frequency, run

```bash
rostopic pub /simple_py_subscriber/subs_str std_msgs/String "data: 'Hello World'" -r 0.2
```

This would publish a message with data as `Hello World` every 5 seconds (0.2 Hz is the rate passed).

You may even experiment with remapping arguments to make the publisher that we had created earlier to publish messages to `/simple_py_subscriber/subs_str` instead of its default programmed publishing topic `/simple_py_publisher/hello_str`. To do that, run the following command

```bash
rosrun py_basic_nodes simple_publisher.py /simple_py_publisher/hello_str:=/simple_py_subscriber/subs_str
```

Remember that the publisher actually publishes at a programmed rate of 0.5 Hz. If you have the previous `rostopic pub` node still running, there actually are two nodes publishing and one node subscribing to the topic. Run the following command

```bash
rosrun rqt_graph rqt_graph
```

Would produce the following output

![rqt_graph subscriber output](./media/pic1.png)

You may further experiment with the code to see how the output changes. Most notably, you can do the following

1. Try creating a large enough delay in the subscriber callback that messages accumulate. Then see what happens. More about creating a delay in Python [here](https://realpython.com/python-sleep/#adding-a-python-sleep-call-with-timesleep). You can further experiment here with different queue sizes.

### Simple Service Server

| Field | Value |
| :--- | :---- |
| Node name | `simple_py_service_server` |
| Code | [scripts/simple_service_server.py](./scripts/simple_service_server.py) |
| Service | [AddAllFloat64Numbers_py.srv](./srv/AddAllFloat64Numbers_py.srv) |

Before this node, you have to understand declaring and building services. Check [AddAllFloat64Numbers_py](#addallfloat64numbers_py) for that. This node demonstrates how to create a service server.

#### Building

In the `CMakeLists.txt`, add the following lines at appropriate places

1. Add the script to `catkin_install_python` function

    Add the line

    ```txt
        scripts/simple_service_server.py
    ```

    before the `DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}` line.

Then, run `catkin_make` in the workspace folder.

#### Running

To run this node, run `roscore` first. To run the node, run

```bash
rosrun py_basic_nodes simple_service_server.py
```

After this, the `rosservice` tool can be used to find if the service is running

```bash
rosservice list
```

To call the service, a service client could be used or the call can also be made through the command line. Try running

```bash
rosservice call /simple_py_service_server/add_numbers "data:
- 12.5
- 37.25
- 100.75
- -20.3"
```

When you used tab completion, the first data value would be filled with 0. Just remove the ending `"` and continue on by typing an enter (or even `Ctrl-V Ctrl-J` would do). Usually after pressing enter, another prompt shows up to indicate completing the string, that is stored in `$PS2` environment variable (usually a `>`). After pressing enter upon completing the string with `"`, an output like this must appear

```txt
sum: 130.2
```

### Simple Service Client

| Field | Value |
| :--- | :---- |
| Node name | `simple_py_service_client` |
| Code | [scripts/simple_service_client.py](./scripts/simple_service_client.py) |
| Service | [AddAllFloat64Numbers_py.srv](./srv/AddAllFloat64Numbers_py.srv) |

Before this node, you have to understand declaring and building services. Check [AddAllFloat64Numbers_py](#addallfloat64numbers_py) for that. This node demonstrates how to create a service client. It will call a service, so the service server needs to be present first. A simple service server is [demonstrated above](#simple-service-server).

#### Building

In the `CMakeLists.txt`, add the following lines at appropriate places

1. Add the script to `catkin_install_python` function

    Add the line

    ```txt
        scripts/simple_service_client.py
    ```

    before the `DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}` line.

Then, run `catkin_make` in the workspace folder.

#### Running

To run this node, run `roscore` first. To use this node, a service server (servicing the service mentioned in the node, that is bearing the name `/simple_py_service_server/add_numbers`) must be running. This package has the [corresponding service server node](#simple-service-server). To run that node, run

```bash
rosrun py_basic_nodes simple_service_server.py
```

Ensure that the service `/simple_py_service_server/add_numbers` is running (using `rosservice list`)

To run this node, run a command like what's shown below (you can use any numbers in the argument)

```bash
rosrun py_basic_nodes simple_service_client.py 60 4 0.5 -98.7
```

If everything's gone right, the sum must return `-34.2` for the given numbers.

## Services

Services declared in this package

### AddAllFloat64Numbers_py

| Field | Value |
| :---- | :---- |
| Name | `AddAllFloat64Numbers_py` |
| File | [srv/AddAllFloat64Numbers_py.srv](./srv/AddAllFloat64Numbers_py.srv) |
| Service Request | `float64[] data` |
| Service Response | `float64 sum` |

A service that adds all the numbers sent in the _request_ and sends the result as a _response_. For example, if we send `1, -2.3, 56, -4` in request, it will send `50.7` (their sum) in response. However, note that the logic is not in the `.srv` file, but in the service server. This file only describes the structure.

#### Building services and messages

The file description is actually just created a service description file. We need to build the files (header and source files) so that this and other packages can use this service.

##### Package.xml

To do that, open `package.xml` file and add the following lines at appropriate places:

1. Add a `build_depend` to `message_generation`. This will allow building messages, services and actions at compile time.

    Go to the part where `<build_depend>` tags are located and add the following tags to the list

    ```xml
    <build_depend>message_generation</build_depend>
    <build_depend>std_msgs</build_depend>
    ```

    This would tell that the package depends on `message_generation` package for building. The package `message_generation` is used for building the messages, services and actions. The second package, `std_msgs`, is needed for the messages.
2. Add an `exec_depend` to `message_runtime`.

    Go to the part where `<exec_depend>` tags are located and add the following tag to the list

    ```xml
    <exec_depend>message_runtime</exec_depend>
    <exec_depend>std_msgs</exec_depend>
    ```

    This is used to tell that we need the `message_runtime` package at runtime. The second package, `std_msgs`, is needed for the messages.

##### CMakeLists.txt

As the `package.xml` file is [just a manifesto file](http://wiki.ros.org/catkin/package.xml), we need to make the following changes to the `CMakeLists.txt` file (as that is the file which is [used to actually build the package](http://wiki.ros.org/catkin/CMakeLists.txt))

1. In the `find_package` function, add `message_generation` to `catkin REQUIRED COMPONENTS` in a new line (after the end of already declared packages)

    After adding `message_generation` (and adding `std_msgs` for standard messages), the function must look somewhat like this

    ```txt
    find_package(catkin REQUIRED COMPONENTS
      rospy
      message_generation
      std_msgs
    )
    ```

    > More about the `find_package` function [here](http://wiki.ros.org/catkin/CMakeLists.txt#Finding_Dependent_CMake_Packages)

2. Under `catkin specific configuration` heading (just before the `Build` heading), add `message_runtime` as `CATKIN_DEPENDS`.

    After adding `message_runtime` (and `std_msgs` for standard messages), the function must look somewhat like this

    ```txt
    catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES py_basic_nodes
    CATKIN_DEPENDS rospy message_runtime std_msgs
    #  DEPENDS system_lib
    )
    ```

    > More about the `catkin_package` function [here](http://wiki.ros.org/catkin/CMakeLists.txt#catkin_package.28.29)

3. Go to the `Declare ROS messages, services and actions` heading. You shall also find the whole procedure as comments in this section.
    1. Add the `add_action_files` (you could uncomment the existing code, or write a new one under it).

        Include the `.srv` files declared in the `srv` folder here. After adding the function, it would look like this

        ```txt
        add_service_files(
            FILES
            AddAllFloat64Numbers_py.srv
        )
        ```

    2. Add the `generate_messages` function to the macro. This is to invoke code generation for the source code.

        Add the `std_msgs` dependency. After everything, the function must look like this

        ```txt
        generate_messages(
            DEPENDENCIES
            std_msgs    # Float64
        )
        ```

    > There is an [example on roswiki](http://wiki.ros.org/catkin/CMakeLists.txt#Example) to demonstrate changes to the CMakeLists.txt file for adding custom messages, services and actions

After this, run `catkin_make` in the workspace directory.

After a successful build, the python modules (that can be imported) can be found in the `devel/lib/python3/dist-packages` directory inside the workspace directory. It must be inside a folder named `py_basic_nodes`. Within this folder, you must find a `srv` folder (which is a common folder for all services generated by the package).

You can also see the header files for C++ generated in the `devel/include` folder inside the workspace folder. These header files can be included in nodes programmed using C++.

For any IDE, you may want to include the `devel/lib/python3/dist-packages` directory (for auto completion features).

## Reference

- roswiki
    - [Remapping Arguments](http://wiki.ros.org/Remapping%20Arguments)
