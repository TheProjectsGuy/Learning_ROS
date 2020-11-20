# INFO 0 - Basic Terminology

This file contains definitions of basic terms used in ROS. Each term is as a header and contains links to additional information.

## Table of contents

- [INFO 0 - Basic Terminology](#info-0---basic-terminology)
    - [Table of contents](#table-of-contents)
    - [Introductory terms](#introductory-terms)
        - [ROS](#ros)
        - [ROS Wiki](#ros-wiki)
        - [File System](#file-system)
            - [Workspace](#workspace)
            - [Packages](#packages)
    - [Concept terms](#concept-terms)
        - [Master](#master)
        - [Nodes](#nodes)
        - [Messages](#messages)
        - [Topics](#topics)
        - [Services](#services)
    - [Auxiliary information](#auxiliary-information)
    - [References](#references)

## Introductory terms

Terms for beginners

### ROS

ROS (Robot Operating System) is a collection of libraries, development tools and processes. It is all packaged in a single software solution. ROS requires a host operating system to run, it is therefore called a *Meta Operating System*.

The objective of ROS is to provide tools for building, simulating as well as deploying robots. It is also designed with compartmentalized development in mind. You can break down a difficult task into simple tasks, then create programs for individual and specific tasks, and then run them all.

> More about ROS [here](http://wiki.ros.org/ROS/Introduction)

ROS is open source, that is all the source code is online. It also had a vibrant community around it for support and enhancements.

### ROS Wiki

This is the ultimate reference manual for anything ROS related. You can visit it through the link [http://wiki.ros.org/](http://wiki.ros.org/).

### File System

ROS has its own manner of organizing files. In this section, some concepts pertaining to that are explored.

#### Workspace

All ROS related development work happens in one particular folder, named the **workspace**. This is where you store all the things related to ROS: executables, build files, source code, etc. This is the single goto folder on your system. Usually, people name it as `~/catkin_ws` (`~/` implying that it is in the home directory). But **catkin** is actually a build tool (it generates executables from the written code).

Every workspace has a `devel/setup.bash` file. Running it with `source` command *activates* it (it's also called sourcing the workspace). It basically tells ROS that "this is the folder that contains user developed packages". You can see the status of ROS path using the environment variable `ROS_PACKAGE_PATH`.

However, in some cases, people may choose to have discrete work done in separate workspaces. In such cases, it is important that at a time, you have only one workspace sourced (don't work on multiple workspaces at once).

> More about ROS Workspaces [here](http://wiki.ros.org/catkin/workspaces)

You can make your own workspace using catkin.

> Tutorial on creating your own workspace [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

#### Packages

Keeping all the work discrete compartments is important for distributing the workload. A package is a folder that does exactly that. Packages are what actually contain all the source material. A workspace may have many packages. This repository is designed as a workspace and also has multiple packages. The smallest thing you can create and share is a package. You may decide to create a package to hold descriptions of a robot. Maybe another to hold different types of executables (nodes) that are required for the robot. Many sensors have a package of their own.

> More about packages [here](http://wiki.ros.org/Packages)

Not any folder can be a package, it needs to meet a few criterion

- It must be within a workspace. By default, within the `src` folder of the workspace
- It must contain a `CMakeLists.txt` file that explains how it must be built, its dependencies, etc. It is for the build system
    > More about _CMakeLists.txt_ file [here](http://wiki.ros.org/catkin/CMakeLists.txt)
- It must contain a `package.xml` file for explaining the properties of the package (version number, maintainer information, dependencies, etc.). It is also called the **package manifest** file.
    > More about _package.xml_ file [here](http://wiki.ros.org/catkin/package.xml)

Multiple packages can also be grouped as a single package. This kind of package is called a **Metapackage**. They have only `package.xml` and `CMakeLists.txt` files.

> More about _metapackages_ [here](http://wiki.ros.org/catkin/package.xml)

## Concept terms

Brief information about specific concepts

### Master

When running a ROS application, the central thing that coordinates all information about the system is called the *Master*. You can run it using the command `roscore`. It manages connections to various nodes, performs logging, cleanup, etc. and maintains an information server is called the parameter server.

> More about ROS master [here](http://wiki.ros.org/Master)

You can also maintain a dictionary of shared variables that can be accessed by different nodes. The ROS master maintains the dictionary which is called **Parameter Server**.

> More about Parameter Server [here](http://wiki.ros.org/Parameter%20Server)

The main job of the Master is managing the **ROS Computational Graph**. It is a virtual graph which manages connections and relations between various processes using ROS.

### Nodes

Executables are called nodes. Code written in packages is converted into an executable using a build process.

> More about nodes [here](http://wiki.ros.org/Nodes)

ROS uses catkin (a tool that uses cmake) to build packages within a workspace.

> More about catkin [here](http://wiki.ros.org/catkin)

Nodes can be programmed using different programming languages. ROS provides development libraries for C++ and Python (called `roscpp` and `rospy`, these are officially supported). Such development libraries that provide tools for language support are called **Client Libraries**. It is just a collection of APIs to ease the job of programming nodes. Alongside C++, Python and Lisp, there are many experimentally developed (some by the community) client libraries for languages like C#, .NET, R, Ruby, Java, JS, MATLAB, etc.

> More about client libraries [here](http://wiki.ros.org/Client%20Libraries)

### Messages

A message is the simplest packet of information that can be transmitted. Nodes communicate with each other using messages. From something so primitive as a string, to something so sophisticated as a path (or a point cloud), everything can be expressed as a single message. In simple words, messages carry around data.

> More about messages [here](http://wiki.ros.org/Messages)

ROS comes with many standard messages. However, if needed, we can create our own messages as well. To define a message, a `.msg` file is created. It is usually put inside a folder named `msg` withing a package.

> More about *msg* files [here](http://wiki.ros.org/msg)

### Topics

Topics are a way of organizing and directing messages. Think of them as channels for messages. It is important to know that any node can be made put messages in a topic and any node can be made to read messages from a topic. It is possible for a topic to exist even when there are nodes only putting messages on it, but no node that is reading from it (vice versa is also true).

Nodes putting messages on a topic are called **publishers** and those that read messages from a topic are called **subscribers**.

> More about topics [here](http://wiki.ros.org/Topics)

### Services

Topics and messages ensure unidirectional asynchronous communication among nodes. However sometimes, a more synchronized bidirectional, request and response, communication is needed. A service allows you to establish a request and response communication model among nodes. Services allow a pair of messages to be defined and exchanged. One for a request and one for a response.

There can be a node that registers to the master as a **Service Server** (which provides the service), and there can be nodes which are **service clients** (which use the service by calling the server).

> More about services [here](http://wiki.ros.org/Services)

Many packages that come with ROS have their own services. However, if needed, we can create our own services as well. To define a service, a `.srv` file is created. It is usually put inside a folder named `srv` within a package.

> More about *srv* files [here](http://wiki.ros.org/srv)

## Auxiliary information

| S. No. | Item | Description |
| :--- | :---- | :---- |
| 1 | [ROS Naming Conventions](http://wiki.ros.org/Names) | Proper naming methods |
| 2 | [rosbash](http://wiki.ros.org/rosbash) | Useful bash utilities when working with ROS |
| 3 | [ROS Actions](http://wiki.ros.org/actionlib) | Allow a Goal, Feedback and Result type of communication between a server and a client |
| 4 | [ROS Technical Overview](http://wiki.ros.org/ROS/Technical%20Overview) | _Advanced_: A technical overview of how different things are actually implemented in ROS |

## References

- [ROS Concepts](http://wiki.ros.org/ROS/Concepts)
