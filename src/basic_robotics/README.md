# Basic Robotics

Basic Robotics related software provided in ROS. This package includes a brief overview of facilities like visualization, robotic simulation and sensors that ROS provides. Essential softwares like `RViz`, `Gazebo` and frameworks like `MoveIt!` are explored here.

## Table of contents

- [Basic Robotics](#basic-robotics)
    - [Table of contents](#table-of-contents)
    - [Creating this package](#creating-this-package)
    - [Foreword](#foreword)
    - [Tutorials](#tutorials)
        - [Tutorial 0: Getting Started with RViz](#tutorial-0-getting-started-with-rviz)
        - [Tutorial 1: Visualizing data in RViz](#tutorial-1-visualizing-data-in-rviz)
        - [Tutorial 2: Building and visualizing robot models](#tutorial-2-building-and-visualizing-robot-models)
            - [Part 1: Visualizing a single link](#part-1-visualizing-a-single-link)
            - [Part 2: Jogging a two link manipulator](#part-2-jogging-a-two-link-manipulator)
        - [Tutorial 3: Gazebo and a Four Wheel Robot](#tutorial-3-gazebo-and-a-four-wheel-robot)
        - [Tutorial 4: Sensors in Gazebo](#tutorial-4-sensors-in-gazebo)
        - [Tutorial 5: Simple 1R Robot](#tutorial-5-simple-1r-robot)
    - [Launch Files](#launch-files)
        - [Launch C++ for Tutorial 1](#launch-c-for-tutorial-1)
        - [Launch Python for Tutorial 1](#launch-python-for-tutorial-1)
        - [Launch single link for Tutorial 2](#launch-single-link-for-tutorial-2)
        - [Visualize Jogging URDF robot for Tutorial 2](#visualize-jogging-urdf-robot-for-tutorial-2)
        - [Visualize and jog a URDF for Tutorial 3](#visualize-and-jog-a-urdf-for-tutorial-3)
        - [Launch Gazebo World for Tutorial 3](#launch-gazebo-world-for-tutorial-3)
        - [Spawn a Robot Model in Gazebo for Tutorial 3](#spawn-a-robot-model-in-gazebo-for-tutorial-3)
        - [Launch Gazebo World for Tutorial 4](#launch-gazebo-world-for-tutorial-4)
        - [Spawn Robot Model for Tutorial 4](#spawn-robot-model-for-tutorial-4)
        - [Visualize robot and data for Tutorial 4](#visualize-robot-and-data-for-tutorial-4)
        - [Gazebo world for Tutorial 5](#gazebo-world-for-tutorial-5)
        - [Spawn robot in Gazebo for Tutorial 5](#spawn-robot-in-gazebo-for-tutorial-5)
        - [Visualize robot for Tutorial 5](#visualize-robot-for-tutorial-5)
    - [C++ Nodes](#c-nodes)
        - [Laser Scan Publisher (C++) for Tutorial 1](#laser-scan-publisher-c-for-tutorial-1)
            - [Building](#building)
            - [Running](#running)
        - [TF Publisher (C++) for Tutorial 1](#tf-publisher-c-for-tutorial-1)
            - [Building](#building-1)
            - [Running](#running-1)
        - [Marker and Static TF Publisher (C++) for Tutorial 1](#marker-and-static-tf-publisher-c-for-tutorial-1)
            - [Building and Running](#building-and-running)
    - [Gazebo Plugins](#gazebo-plugins)
        - [Single Link Robot Controller for Tutorial 5](#single-link-robot-controller-for-tutorial-5)
            - [Building](#building-2)
    - [Python Nodes](#python-nodes)
        - [Laser Scan Publisher (Python) for Tutorial 1](#laser-scan-publisher-python-for-tutorial-1)
            - [Building](#building-3)
            - [Running](#running-2)
        - [TF Publisher (Python) for Tutorial 1](#tf-publisher-python-for-tutorial-1)
            - [Building](#building-4)
            - [Running](#running-3)
        - [Marker and Static TF Publisher (Python) for Tutorial 1](#marker-and-static-tf-publisher-python-for-tutorial-1)
            - [Building and Running](#building-and-running-1)
        - [Robot Visualization (Python) for Tutorial 4](#robot-visualization-python-for-tutorial-4)
    - [Configuration files](#configuration-files)
        - [Gazebo to Robot configurations for Tutorial 4](#gazebo-to-robot-configurations-for-tutorial-4)
        - [Configurations for Tutorial 5](#configurations-for-tutorial-5)
    - [RViz configuration files](#rviz-configuration-files)
        - [LaserScan and TF for Tutorial 1](#laserscan-and-tf-for-tutorial-1)
        - [RobotDescription and TF for Tutorial 2](#robotdescription-and-tf-for-tutorial-2)
        - [RobotDescription and TF for Tutorial 3](#robotdescription-and-tf-for-tutorial-3)
        - [Robot visualization with sensor and TF for Tutorial 4](#robot-visualization-with-sensor-and-tf-for-tutorial-4)
        - [Robot visualization for Tutorial 5](#robot-visualization-for-tutorial-5)
    - [URDF Files](#urdf-files)
        - [Single block for Tutorial 2](#single-block-for-tutorial-2)
        - [Two Blocks for Tutorial 2](#two-blocks-for-tutorial-2)
            - [Checking URDFs](#checking-urdfs)
    - [XACRO Files](#xacro-files)
        - [Simple Four Wheel Bot for Tutorial 3](#simple-four-wheel-bot-for-tutorial-3)
        - [Four Wheel Bot for Gazebo for Tutorial 3](#four-wheel-bot-for-gazebo-for-tutorial-3)
        - [Four Wheel Bot for Gazebo for Tutorial 4](#four-wheel-bot-for-gazebo-for-tutorial-4)
        - [Single link robot for Tutorial 5](#single-link-robot-for-tutorial-5)
    - [Gazebo World Files](#gazebo-world-files)
        - [Four Wheel Bot World for Tutorial 3](#four-wheel-bot-world-for-tutorial-3)
    - [Reference](#reference)

## Creating this package

This package was created using the following commands

```bash
cd ~/ros_workspaces/learning_ws/src
catkin_create_pkg basic_robotics
```

The dependencies will be added later

## Foreword

As this package deals with more practical things, there is relatively less code in this and more of tutorial like exercises. When reading this file, you may either traverse from top to bottom (recommended for beginners) or navigate through [table of contents](#table-of-contents). If you are a beginner, it may be more beneficial to go through the [tutorials](#tutorials).

## Tutorials

Short tutorials included in this package made to cover essential concepts. They are as follows

| S. No. | Name | Notes |
| :--- | :--- | :--- |
| 1 | [Getting Started with RViz](#tutorial-0-getting-started-with-rviz) | Launching RViz and terminologies |
| 2 | [Visualizing data in RViz](#tutorial-1-visualizing-data-in-rviz) | Visualize `TF`, `Marker` and `LaserScan` using dummy publishers |
| 3 | [Building and Visualizing Robot Models](#tutorial-2-building-and-visualizing-robot-models) | Building a simple robot using `URDF`, then visualizing it in RViz using `RobotModel` |
| 4 | [Gazebo and a Four Wheel Robot](#tutorial-3-gazebo-and-a-four-wheel-robot) | Creating a four wheel robot and simulating it in Gazebo |
| 5 | [Sensors in Gazebo](#tutorial-4-sensors-in-gazebo) | Adding sensors and visualizing them in RViz |
| 6 | [Single Joint robot in Gazebo](#tutorial-5-simple-1r-robot) | Creating a custom plugin for controlling a 1R robot in Gazebo |

### Tutorial 0: Getting Started with RViz

RViz is essentially a node that was created to visualize different aspects of robotics. Things like visualizing sensor data (like a Lidar scan, point cloud, path, and even cameral feeds), robot data (like robot models and frame transformations). It is a node that has its GUI made using Qt.

In this tutorial, we will see how to launch it, learn about what is a configuration file and some basic terminologies.

> Rviz: A *visualization* tool in ROS

To start, first run `roscore` (as this is a node, it will require a running ROS Master). To start rviz, run

```bash
rosrun rviz rviz
```

Even simply `rviz` would run it (it is saved as an executable in `/opt/ros/noetic/bin`). After the GUI opens, you may something like this.

![RViz GUI](./media/pic1.png)

Whatever you see in the GUI is loaded by rviz using a *configuration file*. We shall later see how we can create our own configuration files that will help us set up things specific to visualization tasks. For now, see the definition.

> RViz configuration file: A YAML file that contains the type of things RViz should display.

You may see the default configuration file `/opt/ros/noetic/share/rviz/default.rviz` to understand the basic structure (but that won't be required as it is handled by rviz, we virtually never have to edit configuration files directly).

To see different things that Rviz can visualize, click on `Add` (right above `Time` in the bottom left). A menu like what's shown below shall open

![RViz Visualization display types](./media/pic2.png)

This menu includes all the things that come with `rviz` to visualize. You can see the description of each item written under. You may even add a custom *Display Name* to show in the Displays panel (the left panel in RViz GUI). Information on all the built in display types can be found [here](https://wiki.ros.org/rviz/DisplayTypes). Try adding `RobotModel` as shown (click `OK`), you'll see that a new entry in `Displays` has appeared (for now, an error status may be being shown, that's not an issue: it's because we haven't yet created a robot model to visualize). The same will happen for each of the times in the above menu. For each item, there are configuration options, these too are configured as YAML values in the configuration file.

Most importantly, you may notice a topic to choose for many sensors (like `Camera` will have an `Image Topic`). This basically will link that topic: RViz will subscribe to that topic, receive data from publisher and show whatever it gets in the visualization form (an image stream for camera).

Now that we know what RViz is and basics of a configuration file, you may simply close the Rviz GUI window. Choose to close without saving the changes.

### Tutorial 1: Visualizing data in RViz

In this tutorial, we explore how to visualize data on a topic in RViz. For this particular example, we will visualize `TF` (frame transformations) and a `LaserScan`. In the real world, sensor data is directly published by sensors (using hardware plugins for ROS). It is also possible to fetch this data from a simulator like `Gazebo` (more on that later). Here, we will create a dummy node that will publish this data on a topic and then we will configure RViz to subscribe to these topics.

This tutorial uses the following resources of this package

| S. No. | File / Node name | Purpose | Notes |
| :--- | :--- | :---: | :---- |
| 1 | [LaserScan_TF_T1](#laserscan-and-tf-for-tutorial-1) | RViz configuration file | The configuration file consisting of `LaserScan` and `TF` display and an `Axes` |
| 2a | [t1_cpp_laser_scan_publisher](#laser-scan-publisher-c-for-tutorial-1) | C++ Publisher | Dummy publisher for `LaserScan` |
| 2b | [t1_laser_scan_publisher.py](#laser-scan-publisher-python-for-tutorial-1) | Python Publisher | Dummy publisher for `LaserScan` |
| 3a | [t1_cpp_tf_broadcaster](#tf-publisher-c-for-tutorial-1) | C++ Transform Broadcaster | Dummy publisher / broadcaster for `TF` |
| 3b | [t1_tf_publisher.py](#tf-publisher-python-for-tutorial-1) | Python Transform Broadcaster | Dummy publisher / broadcaster for `TF` |
| 4a | [t1_cpp_markers_publisher](#marker-and-static-tf-publisher-c-for-tutorial-1) | C++ Marker and Static Transform publisher | Dummy publisher for `/tf_static` and a sphere marker in RViz |
| 4b | [t1_markers_publisher.py](#marker-and-static-tf-publisher-python-for-tutorial-1) | Python Marker and Static Transform publisher | Dummy publisher for `/tf_static` and a sphere marker in RViz |
| 5a | [t1_cpp_everything.launch](#launch-c-for-tutorial-1) | Launch file (C++) | Launch file for launching all C++ nodes described above |
| 5b | [t1_py_everything.launch](#launch-python-for-tutorial-1) | Launch file (Python) | Launch file for launching all Python nodes described above |

To run everything in this tutorial, use the command

```bash
roslaunch basic_robotics t1_cpp_everything.launch
```

If you prefer on using `Python` instead of `C++`, then substitute `t1_cpp_everything.launch` with `t1_py_everything.launch` in the commands.

This must open an RViz window as shown below

![RViz output for launch file of tutorial 1](./media/pic7.png)

As you must have noticed, the laser scan is now attached to frame `f2` instead of `global` (the default configured in the code). If you want to attach it back to `global`, then you could delete the `<param>` in the launch file, or you could more efficiently use the parameter mechanism that was created. Run the following command (after closing the previous launch by closing the RViz GUI)

```bash
roslaunch basic_robotics t1_cpp_everything.launch ls_frame:=global
```

The sensor must now be in the `global` frame as shown below

![RViz output for launch file of tutorial 1, modified ls_frame](./media/pic8.png)

### Tutorial 2: Building and visualizing robot models

In this tutorial, we explore how to create a `RobotModel` and visualize it in RViz. We will also see how to manually jog the robot (that is, visualize different joints rotating). Keep in mind that RViz is not a simulation tool, it is only a visualization tool. It cannot simulate physics (a simulator like Gazebo will be required for that), it can only display what's given to it by the user. In this tutorial, we will build a simple four wheel robot and visualize it. We shall also see how to jog the robot and visualize that through the model as well as tf (move the joints).

This tutorial uses the following resources of this package

| S. No. | File | Purpose | Notes |
| :--- | :--- | :---: | :---- |
| 1 | [RobotViz_T2](#robotdescription-and-tf-for-tutorial-2) | RViz configuration file | The configuration file consisting of `RobotDescription` and `TF` |
| 2 | [single_block_t2.urdf](#single-block-for-tutorial-2) | URDF File 1 | URDF file containing a single link to be displayed |
| 3 | [t2_single_link_viz](#launch-single-link-for-tutorial-2) | Launch File 1 | Launch file for the single link demo |

#### Part 1: Visualizing a single link

Here, we'll visualize a very simple robot (one link only). Run the following `roslaunch` command

```bash
roslaunch basic_robotics t2_single_link_viz.launch
```

This will launch the RViz GUI like shown below

![RViz window for single link](./media/pic9.png)

This is the [single_block_t2.urdf](#single-block-for-tutorial-2) file. This file is actually loaded into the ROS parameter called `robot_description` (check the launch file code). Close this launch and now we shall inspect the jogging of joints.

#### Part 2: Jogging a two link manipulator

Here, we will see how to manually move joints and visualize them moving (that is what is called jogging). There is no simulation physics happening, only visualization. Run the following `roslaunch` command

```bash
roslaunch basic_robotics t2_urdf_bot_viz.launch
```

This must open RViz with the robot (just two links) and must also create a GUI as shown below

![RViz and joint_state_publisher GUI](./media/pic11.png)

This is the rviz GUI and the GUI created by `joint_state_publisher`. The `rqt_graph` GUI must look like this

![rqt_graph GUI](./media/pic12.png)

To understand what's happening, comment out the nodes `joint_state_publisher_gui` and `robot_state_publisher` in the launch and relaunch everything. The window now must be something like this

![Error in RViz in tutorial 2 part 2](./media/pic13.png)

This error is because there is nothing publishing the `/tf` frame transformations. We could create one for the primitive bot that we made, but that's impractical for large sophisticated robots. We also would like a GUI which would allow us to move the joints and inspect what is happening. Doing that using rqt for every robot we make is also hard. Therefore ros has two solutions for these tasks

1. Node `joint_state_publisher_gui` (package name is also the same): This node will parse the `robot_description` ROS parameter from the parameter server, identify the joints, then create a GUI for publishing these joint values on a topic called `/joint_states`. The published messages are not to be confused with `/tf` as these are of type `sensor_msgs/JointState` (essentially an array of joint names, position, velocity, effort, etc.).

    > More about `joint_state_publisher` [here](https://wiki.ros.org/joint_state_publisher) and `joint_state_publisher_gui` [here](https://wiki.ros.org/joint_state_publisher_gui).

2. Node `robot_state_publisher` (package name is also the same): This node will subscribe to a topic called `/joint_state`, read the ROS parameter `robot_description` and then create a forward kinematics model (which can convert joint positions to actual frame transformations). This node, through this model of forward kinematics, publishes `/tf`. It also notices static (fixed) joints and publishes their information on topic `/tf_static` which persist and reduce the load on `/tf` topic.

    > More about `robot_state_publisher` [here](https://wiki.ros.org/robot_state_publisher)

This communication process is observed through the `rqt_graph` GUI shown above. Uncomment the previously commented files and try jogging (moving joints). The output must look similar to this

![RViz jogging joints](./media/pic14.png)

You must also observe the messages on topics `/joint_states` and `/tf` (notice that they are published continuously). Note that you could also create a publisher for `/tf` (like we did in [tutorial 1](#tutorial-1-visualizing-data-in-rviz)) instead of using the joint state and robot state publisher, or create a publisher for `/joint_states` and use `joint_state_publisher` instead of `joint_state_publisher_gui` (creating a joint controller for visualizing).

You may close the files as we will now be exploring a much more sophisticated robot.

### Tutorial 3: Gazebo and a Four Wheel Robot

In this tutorial, we explore basic simulations. A full installation of ROS comes with an open source simulator called [Gazebo](http://gazebosim.org/) (source code on [GitHub](https://github.com/osrf/gazebo)) which enables simulating robots. We explore how to create and control a four wheel robot (with sensors mounted on it) and get real time data from the simulation to visualize in RViz.

This tutorial uses the following resources of this package

| S. No. | File / Node name | Purpose | Notes |
| :--- | :--- | :---: | :---- |
| 1 | [simple_fwb.xacro](#simple-four-wheel-bot-for-tutorial-3) | XACRO file for visualization | A simple version of the four wheel robot. No sensors, gazebo, only the robot with four wheels fixed to the `world`. |
| 2 | [t3_fwb_world.world](#four-wheel-bot-world-for-tutorial-3) | Gazebo World | The gazebo world file where the robots are visualized. |
| 3 | [t3_viz_robot_xacro.launch](#visualize-and-jog-a-urdf-for-tutorial-3) | Launch file (XACRO in RViz) | The launch file to visualize the XACRO robot model in RViz |
| 4 | [t3_gazebo_world.launch](#launch-gazebo-world-for-tutorial-3) | Launch file (Gazebo world) | The launch file for the Gazebo world file |
| 5 | [t3_gz_spawn_xacro.launch](#spawn-a-robot-model-in-gazebo-for-tutorial-3) | Launch file (Spawn robot in Gazebo) | The launch file for spawning a XACRO robot description in Gazebo |

First off, we can start by creating and visualizing a four wheeled robot in RViz. This must be the first step in a robotics application. The following steps can be followed for this purpose (see file names and links for the reference)

1. Create your robot using [XACRO](https://wiki.ros.org/xacro) (which will make writing description code easier)

    1. There are many softwares that create a URDF directly from a CAD model. One example is [SolidWorks 3D CAD](https://www.solidworks.com/domain/design-engineering) using a [URDF exporter add-in](http://wiki.ros.org/sw_urdf_exporter). However, for this tutorial, the links are simple `<geometry>` elements.
    2. Exporting from SolidWorks is also problematic as we will later add `<gazebo>` to the mix (you'll have to manually edit that long URDF generated). A better practice may be to import meshes of the links (for `<visual>` and `<collision>`) and get proper values for `<inertia>` from the CAD software.
    3. Currently, we won't have any _sensors_ mounted on the robot (we'll explore that in later tutorials)

    The file thus created is [simple_fwb.xacro](#simple-four-wheel-bot-for-tutorial-3). Note that this file has nothing for Gazebo yet and that there is a fixed link from `world` (a dummy link) and `body` (the body link).

2. Visualize your robot in RViz. Run the following command

    ```bash
    roslaunch basic_robotics t3_viz_robot_xacro.launch
    ```

    This will create the `RViz` and `joint_state_publisher_gui` GUIs as shown below

    ![Simple four wheel robot in RViz](./media/pic15.png)

Now, we can create a world for Gazebo simulation. This is the robot environment. It contains buildings (usually walls, boundaries, windows, floors, etc.) and models (objects like tables, chairs, pretty much any physical object that you need in the scene).

1. Create the [t3_fwb_world.world](#four-wheel-bot-world-for-tutorial-3) file (the Gazebo world for the simulation environment)
2. Visualize the Gazebo world

    Run the following commands

    ```bash
    roscore
    roscd basic_robotics/world/
    rosrun gazebo_ros gazebo t3_fwb_world.world
    ```

    This should launch the gazebo world as shown below

    ![Gazebo world for Tutorial 3 Part 1](./media/pic20.png)

Now, make your robot Gazebo ready, that is, create `<gazebo>` tags for the links and joints. The usual practice to do this is to create a `.gazebo` file and `<xacro:include ...>` it in your `.xacro` file. This way, things are less cluttered and everything Gazebo related is in one file. Note that though the extension is `.gazebo`, it is mainly to distinguish. The file is still an XML file (XACRO file) and all XACRO features can be used after `xmlns:xacro` is added to the `<robot>` in the file.

1. Create the [fwb_t3.gazebo](./urdf/fwb_t3.gazebo) file
2. Add `<gazebo>` for the robot, different links and joints
3. Add a `<plugin>` for the robot `<gazebo>`. As this is a four wheeled robot (skid steer drive), we use the [SkidSteerDrive](http://gazebosim.org/tutorials?tut=ros_gzplugins&cat=connect_ros#SkidSteeringDrive) plugin for that. If our robot were a two wheeled robot with a castor, it'd be called a Differential drive robot and we'd use the [DifferentialDrive](http://gazebosim.org/tutorials?tut=ros_gzplugins&cat=connect_ros#DifferentialDrive) plugin for the robot.
4. Include this file in the main `XACRO` file. Remember to not have a fixed dummy to the `world` link, else the robot will be a fixed one (immobile, as the base will be immovable). For this purpose, a separate [fwb_gazebo_t3.xacro](./urdf/fwb_gazebo_t3.xacro) file has been created.

Now, you can _spawn_ the robot in Gazebo using the following commands

```bash
roslaunch basic_robotics t3_gazebo_world.launch
roslaunch basic_robotics t3_gz_spawn_xacro.launch
```

Run the second launch after the Gazebo GUI (node name `/gazebo_gui`) fully loads. The second launch will spawn the robot in the gazebo environment. The robot must look like this in the environment.

![Robot in Gazebo for Tutorial 3](./media/pic21.png)

List the topics using `rostopic list`. You must see `/cmd_vel` as one. You could control the robot by publishing values directly on the topic using something like

```bash
rostopic pub /robot1/cmd_vel geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.2"
```

But, there is a package that allows teleoperation through the keyboard called [teleop_twist_keyboard](https://wiki.ros.org/teleop_twist_keyboard). Run it using

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

You can then control the robot using the keyboard like shown below

![Robot control through turtle teleoperation](./media/pic22.png)

### Tutorial 4: Sensors in Gazebo

In this tutorial, we explore adding basic sensors in Gazebo robot models. We explore adding a camera and a laser scanner (LiDAR) to the robot from [tutorial 3](#tutorial-3-gazebo-and-a-four-wheel-robot). We also explore how to get the joint states from the robot in Gazebo and demonstrate how to visualize them in RViz.

This tutorial uses the following resources of this package

| S. No. | File / Node name | Purpose | Notes |
| :--- | :--- | :---: | :---- |
| 1 | [fwb_gazebo_t4.xacro](#four-wheel-bot-for-gazebo-for-tutorial-4) | XACRO File | Robot description in XACRO, gazebo plugin tags, etc. |
| 2 | [t4_gazebo_world.launch](#launch-gazebo-world-for-tutorial-4) | Launch file | Launches the Gazebo world |
| 3 | [t4_gz_spawn_xacro.launch](#spawn-robot-model-for-tutorial-4) | Launch file | Spawns the XACRO file above in the Gazebo world |
| 4 | [t4_robot_joints.py](#robot-visualization-python-for-tutorial-4) | Python Node | Node to translate link states from Gazebo to joint states in RViz |
| 5 | [t4_robot_viz.launch](#visualize-robot-and-data-for-tutorial-4) | Launch file | Visualizes the robot in RViz along with translation of messages |

Borrowing the XACRO from [tutorial 3](#tutorial-3-gazebo-and-a-four-wheel-robot), add the following for each sensor

1. A `<link>` containing the `<inertia>`, `<visual>` and `<collision>` properties. If using CAD files, it is suggested to use simpler meshes for `<collision>` (as that is what is used by the simulator), `<visual>` can be more detailed as it is only what is rendered once in simulation.
2. A `<joint>` for each sensor. Usually most of them are defined as `fixed` joint relative to the base.
3. A `<sensor>` [tag](http://sdformat.org/spec?ver=1.8&elem=sensor) defining the properties of the particular sensor and the [plugin](http://gazebosim.org/tutorials?tut=ros_gzplugins&cat=connect_ros#Pluginsavailableingazebo_plugins) (which will handle all ROS related details of the sensor).

To inspect the robot in simulation, run the following commands

```bash
roslaunch basic_robotics t4_gazebo_world.launch
roslaunch basic_robotics t4_gz_spawn_xacro.launch
```

You can control the robot using

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Note that this node only is to publish messages on `/cmd_vel` topic. This will show the following in the gazebo GUI.

![Gazebo window](./media/pic23.png)

You can visualize everything (the LIDAR scan, camera, robot joint transformations, etc.) using

```bash
roslaunch basic_robotics t4_robot_viz.launch
```

This must open an RViz window like shown below

![RViz for visualizing Tutorial 4](./media/pic24.png)

The corresponding Gazebo GUI is as shown below

![Gazebo GUI for Tutorial 4](./media/pic25.png)

Note that the wheels can be seen rotating in the RViz RobotModel as they are made to rotate by the plugin (Skid Steer Drive plugin) in Gazebo. This is done because of the [translation node](#robot-visualization-python-for-tutorial-4).

### Tutorial 5: Simple 1R Robot

[Plugins in Gazebo](http://gazebosim.org/tutorials?cat=write_plugin) allow you to control finer aspects of your model. Gazebo also offers [ROS APIs for custom plugins](http://gazebosim.org/tutorials?tut=ros_gzplugins&cat=connect_ros). In this tutorial, we create a single joint robot which we control using a custom C++ plugin. The controller is a simple PD controller. We also visualize the robot in RViz.

This tutorial uses the following resources of this package

| S. No. | File / Node name | Purpose | Notes |
| :--- | :--- | :---: | :---- |
| 1 | [slbot_t5.xacro](#single-link-robot-for-tutorial-5) | XACRO file | Robot description in XACRO and the gazebo tags for the plugin created |
| 2 | [t5_gazebo_world.launch](#gazebo-world-for-tutorial-5) | Launch file | Launch file for Gazebo world |
| 3 | [t5_gz_spawn_xacro.launch](#spawn-robot-in-gazebo-for-tutorial-5) | Launch file | Launch file to spawn the robot in Gazebo |
| 4 | [t5_viz_robot.launch](#visualize-robot-for-tutorial-5) | Launch file | Launch file for visualizing the robot |
| 5 | [libgazebo_ros_slbot_controller](#single-link-robot-for-tutorial-5) | Gazebo plugin | Custom made Gazebo plugin for PD control of a single axis joint (the robot in this tutorial) |

To inspect the robot in this tutorial, run

```bash
roslaunch basic_robotics t5_viz_robot.launch use_gui:=true
```

This should open RViz with the GUI for jogging the robot link shown below

![Visualize and jog robot for Tutorial 5](./media/pic26.png)

Close this launch and launch the following commands

```bash
roscore
roslaunch basic_robotics t5_gazebo_world.launch
roslaunch basic_robotics t5_gz_spawn_xacro.launch
```

This should create the following mode in the gazebo world

![Robot Model in Gazebo](./media/pic27.png)

Get the services and topics using

```bash
rosservice list
rostopic list
```

You must see service `/sl_robot1/set_pid` and topics `/sl_robot1/cmd_pos` and `/sl_robot1/joint_states`. The service is to tune PID constants for the robot. The first topic is to send the command positions (target positions) and the second topic is published by the plugin (for visualizing joint angles). This is the better way of visualizing joint angles.

Try running

```bash
roslaunch basic_robotics t5_viz_robot.launch
rqt_graph
```

Note that now, the GUI isn't there. The `rqt_graph` shows Gazebo sending the JointStates to a joint state publisher which then sends it to a robot state publisher. The robot state publisher publishes the `/tf` transformations. You can give the robot target positions using

```bash
rostopic pub /sl_robot1/cmd_pos std_msgs/Float64 "data: 0.785398" -1
```

You can adjust the PID constants using

```bash
rosservice call /sl_robot1/set_pid "{p: 50.0, i: 0.0, d: 15.0, i_clamp: 0.0, antiwindup: false}"
```

Here are the outputs of a random configuration (windows shown side by side)

![Side by side simulation and visualization](./media/pic28.png)

Here, the simulation and visualization are happening sided by side.

## Launch Files

### Launch C++ for Tutorial 1

| Field | Value |
| :---- | :---- |
| Name | `t1_cpp_everything` |
| File | [launch/t1_cpp_everything.launch](./launch/t1_cpp_everything.launch) |

Launches all the C++ Nodes for tutorial 1, along with RViz and TF visualization node. The launch file launches the following nodes

1. [t1_cpp_tf_broadcaster](#tf-publisher-c-for-tutorial-1)
2. [t1_cpp_laser_scan_publisher](#laser-scan-publisher-c-for-tutorial-1)
3. [t1_cpp_markers_publisher](#marker-and-static-tf-publisher-c-for-tutorial-1)
4. RViz with the configuration file for the tutorial: [LaserScan_TF_T1](#laserscan-and-tf-for-tutorial-1)
5. Node `rqt_tf_tree` of package `rqt_tf_tree`

### Launch Python for Tutorial 1

| Field | Value |
| :---- | :---- |
| Name | `t1_py_everything` |
| File | [launch/t1_py_everything.launch](./launch/t1_py_everything.launch) |

Launches all the Python Nodes for tutorial 1, along with RViz and TF visualization node. The launch file launches the following nodes

1. [t1_py_laser_scan_publisher](#laser-scan-publisher-python-for-tutorial-1)
2. [t1_py_tf_broadcaster](#tf-publisher-python-for-tutorial-1)
3. [t1_py_markers_publisher](#marker-and-static-tf-publisher-python-for-tutorial-1)
4. RViz with the configuration file for the tutorial: [LaserScan_TF_T1](#laserscan-and-tf-for-tutorial-1)
5. Node `rqt_tf_tree` of package `rqt_tf_tree`

### Launch single link for Tutorial 2

| Field | Value |
| :---- | :---- |
| Name | `t2_single_link_viz` |
| File | [launch/t2_single_link_viz.launch](./launch/t2_single_link_viz.launch) |

Launch a single link on RViz. Includes the [RobotViz_T2.rviz](#robotdescription-and-tf-for-tutorial-2) `rviz` file launch, and sets the `robot_description` parameter to the contents of the URDF file [single_block_t2.urdf](#single-block-for-tutorial-2).

### Visualize Jogging URDF robot for Tutorial 2

| Field | Value |
| :---- | :---- |
| Name | `t2_urdf_bot_viz` |
| File | [launch/t2_urdf_bot_viz.launch](./launch/t2_urdf_bot_viz.launch) |

Launch a URDF robot and visualize it by jogging (moving by rotation or translation, depending on the joint type) different joints. Includes launching the following nodes

1. RViz node with the configuration [RobotViz_T2.rviz](#robotdescription-and-tf-for-tutorial-2)
2. Node `joint_state_publisher_gui` which will create a GUI for us to jog the joints
3. Node `robot_state_publisher` which will produce frame transformations on the topic `/tf`
4. GUI Node `rqt_graph` to see what is happening

### Visualize and jog a URDF for Tutorial 3

| Field | Value |
| :---- | :---- |
| Name | `t3_viz_robot_xacro` |
| File | [launch/t3_viz_robot_xacro.launch](./launch/t3_viz_robot_xacro.launch)

Visualize a robot made using XACRO and jog different joints. Includes launching the following

1. RViz node with the configuration [RobotViz_T3.rviz](#robotdescription-and-tf-for-tutorial-3)
2. Node `joint_state_publisher_gui` which will create a GUI for us to jog the joints
3. Node `robot_state_publisher` which will produce frame transformations on the topic `/tf`
4. GUI node `rqt_graph` to see what is happening
5. GUI node `rqt_tf_tree` to see the transformation tree (all frames in the topic `/tf`)

### Launch Gazebo World for Tutorial 3

| Field | Value |
| :---- | :---- |
| Name | `t3_gazebo_world` |
| File | [launch/t3_gazebo_world.launch](./launch/t3_gazebo_world.launch) |

Launches the Gazebo world for tutorial 3. Includes launching the following

1. Includes the `empty_world.launch` file in `gazebo_ros` package
2. Uses a `gz_world` argument for the `.world` file

The `empty_world.launch` file launches `gzserver` (as node name `gazebo`) and `gzclient` (as node name `gazebo_gui`).

### Spawn a Robot Model in Gazebo for Tutorial 3

| Field | Value |
| :---- | :---- |
| Name | `t3_gz_spawn_xacro` |
| File | [./launch/t3_gz_spawn_xacro.launch](./launch/t3_gz_spawn_xacro.launch) |

Launches a single node that calls `/gazebo/spawn_urdf_model` service (type `gazebo_msgs/SpawnModel`). Has the following nodes

1. Node to spawn a robot URDF model in gazebo using `spawn_model` node of `gazebo_ros` package

The node will get URDF from a ROS parameter

### Launch Gazebo World for Tutorial 4

| Field | Value |
| :---- | :---- |
| Name | `t4_gazebo_world` |
| File | [launch/t4_gazebo_world.launch](./launch/t4_gazebo_world.launch) |

Launches the gazebo world for tutorial 4. Simply includes the [t3_gazebo_world](#launch-gazebo-world-for-tutorial-3) launch file.

### Spawn Robot Model for Tutorial 4

| Field | Value |
| :---- | :---- |
| Name | `t4_gz_spawn_xacro` |
| File | [launch/t4_gz_spawn_xacro.launch](./launch/t4_gz_spawn_xacro.launch) |

Spawns the robot model (in XACRO) in the gazebo world. Includes the [t3_gz_spawn_xacro](#spawn-a-robot-model-in-gazebo-for-tutorial-3) launch file, but with the XACRO for [tutorial 4](#tutorial-4-sensors-in-gazebo).

### Visualize robot and data for Tutorial 4

| Field | Value |
| :---- | :---- |
| Name | `t4_robot_viz` |
| File | [launch/t4_robot_viz.launch](./launch/t4_robot_viz.launch)

Does the following

- Launches `rviz` with configuration [RobotViz_T4.rviz](#robot-visualization-with-sensor-and-tf-for-tutorial-4)
- Launches nodes `joint_state_publisher` and `robot_state_publisher` for generating `/tf` of the robot for visualizing.
- Launches node [t4_robot_joints.py](#robot-visualization-python-for-tutorial-4) for translating messages between Gazebo's LinkState and sensor's JointState. This and the `joint_state_publisher` are given the configuration file [t4_gz_robot_lframes.yaml](#gazebo-to-robot-configurations-for-tutorial-4)
- Defines the `robot_description` parameter in a separate namespace for the `RobotDescription` in RViz. This is for demonstration purposes only.

### Gazebo world for Tutorial 5

| Field | Value |
| :---- | :---- |
| Name | `t5_gazebo_world` |
| File | [launch/t5_gazebo_world.launch](./launch/t5_gazebo_world.launch) |

Simply includes the `empty_world.launch` file for the Gazebo world.

### Spawn robot in Gazebo for Tutorial 5

| Field | Value |
| :---- | :---- |
| Name | `t5_gz_spawn_xacro` |
| File | [launch/t5_gz_spawn_xacro.launch](./launch/t5_gz_spawn_xacro.launch) |

Spawns the robot in the gazebo world for [tutorial 5](#tutorial-5-simple-1r-robot).

### Visualize robot for Tutorial 5

| Field | Value |
| :---- | :---- |
| Name | `t5_viz_robot` |
| File | [launch/t5_viz_robot.launch](./launch/t5_viz_robot.launch) |

Does the following

- Launches `rviz` with configuration [Robot_Viz_T5.rviz](#robot-visualization-for-tutorial-5)
- Launches node `robot_state_publisher`
- Launches node `joint_state_publisher_gui` or `joint_state_publisher` (depending on the `use_gui` argument)

## C++ Nodes

Nodes written in C++ for this package.

### Laser Scan Publisher (C++) for Tutorial 1

| Field | Value |
| :---- | :---- |
| Name | `t1_cpp_laser_scan_publisher` |
| File | [src/t1_laser_scan_publisher.cpp](./src/t1_laser_scan_publisher.cpp) |

This node is a simple publisher that publishes messages of type `sensor_msgs/LaserScan`. It accepts the publishing topic name, frequency and frame from the parameter server (as local parameters) and publishes messages to topic `/t1_laser_scan` (by default). See the code for more information.

#### Building

In your `CMakeLists.txt` file, add

1. In the `find_package` function, add `std_msgs` and `sensor_msgs`. These are needed as dependencies for accessing built message headers. Your function `find_package` in `CMakeLists.txt` file must look somewhat like this

    ```txt
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
    )
    ```

    Notice the dependency on `roscpp` as well. This is how you can add dependencies to a catkin package that was initialized with no dependencies.

2. In the `catkin_package` function (under section `catkin specific configuration`), add the packages `roscpp`, `std_msgs` and `sensor_msgs` as `CATKIN_DEPENDS` because these packages will be needed by others who create projects dependent on our package. It is good to have them so that the dependency tree is created for catkin. The function must end up looking somewhat like this

    ```txt
    catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES basic_robotics
        CATKIN_DEPENDS roscpp std_msgs sensor_msgs
    #  DEPENDS system_lib
    )
    ```

3. Next, add the following lines in your `CMakeLists.txt` file to create the executable

    ```txt
    add_executable(t1_laser_scan_pub src/t1_laser_scan_publisher.cpp)
    target_link_libraries(t1_laser_scan_pub ${catkin_LIBRARIES})
    ```

In your `package.xml` file, add

1. `<build_depend>`, `<build_export_depend>` and `<exec_depend>` for each of `roscpp`, `std_msgs` and `sensor_msgs`. Basically add the following lines at appropriate places

    Add these right after `<buildtool_depend>` (catkin build tool)

    ```xml
    <build_depend>roscpp</build_depend>
    <build_depend>std_msgs</build_depend>
    <build_depend>sensor_msgs</build_depend>
    <build_export_depend>roscpp</build_export_depend>
    <build_export_depend>std_msgs</build_export_depend>
    <build_export_depend>sensor_msgs</build_export_depend>
    <exec_depend>roscpp</exec_depend>
    <exec_depend>std_msgs</exec_depend>
    <exec_depend>sensor_msgs</exec_depend>
    ```

Now, run `catkin_make` in the workspace directory

#### Running

This node will actually be run as a part of tutorial 1, but there are some important things you must infer from just this node.

Run `roscore` first. To run this node, run the command

```bash
rosrun basic_robotics t1_laser_scan_pub
```

You must now see `/t1_laser_scan` in `rostopic list`. Now run rviz using the following commands (the configuration file was made using [this](#laserscan-and-tf-for-tutorial-1))

```bash
roscd basic_robotics
rosrun rviz rviz -d ./rviz/LaserScan_TF_T1.rviz
```

Now, under `LaserScan` in `Displays`, choose the `Topic` to be `/t1_laser_scan`. You'll see the `Status` soon turn to `Error` with the message under `Transform`, as shown below

![RViz error on LaserScan](./media/pic3.png)

The error message basically means that RViz could not find transformation from `global` to `map`. This is because the `Fixed Frame` is set to `map`. Change it to `global` and you'll see the points correctly being rendered (you can change the display properties under `LaserScan` display item). It must look something like this

![RViz LaserScan output](./media/pic4.png)

You may close the `RViz` GUI and save the configurations to the same file. More on this is described in the launch file and the tutorial description.

### TF Publisher (C++) for Tutorial 1

| Field | Value |
| :---- | :---- |
| Name | `t1_cpp_tf_broadcaster` |
| File | [src/t1_tf_publisher.cpp](./src/t1_tf_publisher.cpp) |

This node is a transformation broadcaster. It basically publishes messages of type `tf2_msgs/TFMessage` on the topic `/tf` (which transmits all transformations in the system). The `tf2` framework is a wrapper to make handling transformation efficient.

#### Building

In `CMakeLists.txt` add the following

1. In the `find_package` function, add `tf2` and `tf2_ros`
2. In the `catkin_package` function under `CATKIN_DEPENDS`, add `tf2` and `tf2_ros`
3. Add the following at appropriate places to build the node

    ```txt
    add_executable(t1_tf_broadcaster src/t1_tf_publisher.cpp)
    target_link_libraries(t1_tf_broadcaster ${catkin_LIBRARIES})
    ```

After that, run `catkin_make` in the workspace directory

#### Running

This node is a part of tutorial 1. However, if you want to run this node in isolation, try running these commands and observe the output

Run `roscore`, then start RViz with the [configuration](#laserscan-and-tf-for-tutorial-1) for this tutorial. Then start this node by running

```bash
rosrun basic_robotics t1_tf_broadcaster
```

The output on RViz must appear to be somewhat like this

![Output of TF on RViz](./media/pic5.png)

After running this, you can use `rqt_tf_tree` to inspect the transformation tree at any instant. Run it using

```bash
rosrun rqt_tf_tree rqt_tf_tree
```

The output must be something show a transformation tree somewhat like this

![TF observed in rqt_tf_tree](./media/pic6.png)

### Marker and Static TF Publisher (C++) for Tutorial 1

| Field | Value |
| :---- | :---- |
| Name | `t1_cpp_markers_publisher` |
| File | [src/t1_markers_publilsher.cpp](./src/t1_markers_publilsher.cpp) |

This node publishes static transformations on the `/tf_static` topic. It also publishes a sphere marker. This node is included in the [launch file](#launch-c-for-tutorial-1) for [tutorial 1](#tutorial-1-visualizing-data-in-rviz).

#### Building and Running

In `CMakeLists.txt` add the following

1. In the `find_package` function, add dependency on `geometry_msgs` and `visualization_msgs`. Add them also to the `CATKIN_DEPENDS` list in `catkin_package` function.
2. Add the following lines at appropriate places for building the executables

    ```makefile
    add_executable(t1_markers_publisher src/t1_markers_publilsher.cpp)
    target_link_libraries(t1_markers_publisher ${catkin_LIBRARIES})
    ```

In the `package.xml` file, add `<build_depend>`, `<exec_depend>` and `<build_export_depend>` tags for `geometry_msgs` and `visualization_msgs` package (dependencies).

Build the package using `catkin_make` in the workspace directory. This node is included in the [launch file](#launch-c-for-tutorial-1) for [tutorial 1](#tutorial-1-visualizing-data-in-rviz).

## Gazebo Plugins

### Single Link Robot Controller for Tutorial 5

| Field | Value |
| :---- | :---- |
| Name | `libgazebo_ros_slbot_controller.so` |
| Header file | [gazebo_ros_slbot_controller.h](./include/gazebo_plugins/gazebo_ros_slbot_controller.h) |
| Source code | [gazebo_ros_slbot_controller.cpp](./src/gazebo_ros_slbot_controller.cpp)

This is a Gazebo plugin created to control the single link (single revolute joint) robot demonstrated in [Tutorial 5](#tutorial-5-simple-1r-robot). A plugin is an extension to the Gazebo node that is handled by the gazebo server. Usually, plugins are kept in a separate package, but they can be merged with other things (like here).

#### Building

In your `package.xml`, do the following

1. Add `gazebo_ros` dependency

    ```xml
    <build_depend>gazebo_ros</build_depend>
    <build_export_depend>gazebo_ros</build_export_depend>
    <exec_depend>gazebo_ros</exec_depend>
    ```

    This will be needed for the parent classes (and for Gazebo integration). Add the same tags for all package dependencies, like `std_msgs` and `control_toolbox`.

2. In the `<export> ... </export>` section, that is present just before the `</package>` (insert if missing), a `<gazebo_ros>` is to be added

    The section should become

    ```xml
    <export>
        <!-- Other tools can request additional information be placed here -->
        <gazebo_ros plugin_path="${prefix}/../../lib" gazebo_media_path="${prefix}" />
    </export>
    ```

    The `plugin_path` is actually pointing to the place the library's `.so` file will be placed. It will allow `filename` in `<plugin>` to use only its base name (not the full path which may differ based on system).

In your `CMakeLists.txt`, add the following

1. We'll be using C++17, and to force that, add the following after `project(...)`

    Add the c++17 as a compile option

    ```makefile
    add_compile_options(-std=c++17)
    ```

    This will ensure that C++17 standards are used when compiling (requirement for Gazebo)

2. Add `gazebo_ros` (and other dependent packages) to `find_package(catkin REQUIRED COMPONENTS` list for showing dependency during build
3. Add `gazebo` system dependency. Just after the `find_package` function above, add another function as shown below (there must be a comment section dedicated to it)

    ```makefile
    find_package(gazebo REQUIRED)
    ```

    This will ensure that the system has Gazebo installed before building the packages.

4. Add `include_directories` in the section named `catkin specific configuration` (you may have to create a new section just after the header)

    ```makefile
    # Include directories required for build
    include_directories(include
    ${Boost_INCLUDE_DIR} 
    ${catkin_INCLUDE_DIRS} 
    ${GAZEBO_INCLUDE_DIRS}
    )
    ```

    This will include the header files for boost, catkin and gazebo when building the library. Note that these are only header files, not source code. This is usually the `/opt/ros/noetic/include/gazebo_plugins/` folder.

    Usually, the source code is shared as `.so` in installations. To _link_ that, you'll have to add another similar section for `link_directories`.

    ```makefile
    # Link the directories for Gazebo library
    link_directories(${GAZEBO_LIBRARY_DIRS})
    ```

    This will include the `.so` directories for Gazebo. This is usually the `/opt/ros/noetic/lib` folder.

5. Add the library to the `catkin_package` function in the section `catkin specific configuration`

    First, add the `INCLUDE_DIRS include` line. Then, add `LIBRARIES` and then the `roscpp` under `DEPENDS`. The function must look somewhat like this

    ```makefile
    catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
        gazebo_ros_slbot_controller
    CATKIN_DEPENDS
        roscpp
        rospy 
        std_msgs 
        sensor_msgs 
        tf2 
        tf2_ros 
        geometry_msgs 
        visualization_msgs
    DEPENDS
        roscpp
    )
    ```

    Note that

    - The `INCLUDE` is important as the library is actually consisting of declarations in a separate header file (in the `include` folder).
    - `LIBRARIES` is given a list of libraries that this package is generating. The one we're dealing with is `gazebo_ros_slbot_controller` and will be saved as `libgazebo_ros_slbot_controller` in the `devel/lib` folder of workspace.
    - `DEPENDS` is just for system dependencies.
    - `CATKIN_DEPENDS` is the same from previous tutorials (not actually needed here).

6. In the `Build` section, include the `include` folder

    ```makefile
    include_directories(
        include
        ${catkin_INCLUDE_DIRS}
    )
    ```

    This will include the folder for adding library

7. Add the library using `add_library` function

    ```makefile
    add_library(gazebo_ros_slbot_controller
        src/gazebo_ros_slbot_controller.cpp
    )
    ```

    This will create the `gazebo_ros_slbot_controller` library (as the `libgazebo_ros_slbot_controller.so` file)

8. You can link catkin libraries using

    ```makefile
    target_link_libraries(gazebo_ros_slbot_controller ${catkin_LIBRARIES})
    ```

9. Install the targets for the library

    ```makefile
    install(TARGETS
        gazebo_ros_slbot_controller
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    )
    ```

    This will install the `gazebo_ros_slbot_controller` library to its correct destination. More information on variables [here](https://docs.ros.org/en/jade/api/catkin/html/user_guide/variables.html).

10. Install the C++ header files to the appropriate folder

    ```bash
    install(
        DIRECTORY include/gazebo_plugins
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/gazebo_plugins
    )
    ```

    This simply sets the include path

After this, save and run `catkin_make` in the workspace directory. You must see a `libgazebo_ros_slbot_controller.so` in `devel/lib/` directory in workspace. This piece of code is run by Gazebo, when `<plugin>` is encountered in `<gazebo> ... </gazebo>`. For example, this would call the `Load` function (check source code)

```xml
<gazebo>
    <plugin name="slbot_controller" filename="libgazebo_ros_slbot_controller.so" />
</gazebo>
```

## Python Nodes

### Laser Scan Publisher (Python) for Tutorial 1

| Field | Value |
| :---- | :---- |
| Name | `t1_py_laser_scan_publisher` |
| File | [scripts/t1_laser_scan_publisher.py](./scripts/t1_laser_scan_publisher.py) |

This node is a simple publisher that publishes messages of type `sensor_msgs/LaserScan`. It accepts the publishing topic name, frequency and frame from the parameter server (as local parameters) and publishes messages to topic `/t1_laser_scan` (by default). See the code for more information.

#### Building

In your `CMakeLists.txt` file, add

1. In the `find_package` function, add `std_msgs` and `sensor_msgs`. These are needed as dependencies for accessing built message headers. Your function `find_package` in `CMakeLists.txt` file must look somewhat like this

    ```txt
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sensor_msgs
    )
    ```

    Notice the dependency on `rospy` as well. This is how you can add dependencies to a catkin package that was initialized with no dependencies. The `roscpp` dependency is not required if you do not have any `C++` nodes in your package.

2. In the `catkin_package` function (under section `catkin specific configuration`), add the packages `rospy`, `std_msgs` and `sensor_msgs` as `CATKIN_DEPENDS` because these packages will be needed by others who create projects dependent on our package. It is good to have them so that the dependency tree is created for catkin. The function must end up looking somewhat like this

    ```txt
    catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES basic_robotics
        CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs
    #  DEPENDS system_lib
    )
    ```

3. Next, add the following lines in your `CMakeLists.txt` file to create the executable

    ```txt
    catkin_install_python(PROGRAMS
        scripts/t1_laser_scan_publisher.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    ```

In your `package.xml` file, add

1. `<build_depend>`, `<build_export_depend>` and `<exec_depend>` for each of `rospy`, `std_msgs` and `sensor_msgs`. Basically add the following lines at appropriate places

    Add these right after `<buildtool_depend>` (catkin build tool)

    ```xml
    <buildtool_depend>catkin</buildtool_depend>
    <build_depend>roscpp</build_depend>
    <build_depend>rospy</build_depend>
    <build_depend>std_msgs</build_depend>
    <build_depend>sensor_msgs</build_depend>
    <build_export_depend>roscpp</build_export_depend>
    <build_export_depend>rospy</build_export_depend>
    <build_export_depend>std_msgs</build_export_depend>
    <build_export_depend>sensor_msgs</build_export_depend>
    <exec_depend>roscpp</exec_depend>
    <exec_depend>rospy</exec_depend>
    <exec_depend>std_msgs</exec_depend>
    <exec_depend>sensor_msgs</exec_depend>
    ```

    You do not need `roscpp` if you do not have `C++` nodes in your package.

Now, run `catkin_make` in the workspace directory

#### Running

This node will actually be run as a part of tutorial 1, but there are some important things you must infer from just this node.

Run `roscore` first. To run this node, run the command

```bash
rosrun basic_robotics t1_laser_scan_publisher.py
```

You must now see `/t1_laser_scan` in `rostopic list`. Now run rviz using the following commands (the configuration file was made using [this](#laserscan-and-tf-for-tutorial-1))

```bash
roscd basic_robotics
rosrun rviz rviz -d ./rviz/LaserScan_TF_T1.rviz
```

Now, under `LaserScan` in `Displays`, choose the `Topic` to be `/t1_laser_scan`. You'll see the `Status` soon turn to `Error` with the message under `Transform` (that is if you haven't applied any changes to the RViz configuration file), as shown below

![RViz error on LaserScan](./media/pic3.png)

The error message basically means that RViz could not find transformation from `global` to `map`. This is because the `Fixed Frame` is set to `map`. Change it to `global` and you'll see the points correctly being rendered (you can change the display properties under `LaserScan` display item). It must look something like this

![RViz LaserScan output](./media/pic4.png)

You may close the `RViz` GUI and save the configurations to the same file. More on this is described in the launch file and the tutorial description.

### TF Publisher (Python) for Tutorial 1

| Field | Value |
| :---- | :---- |
| Name | `t1_py_tf_broadcaster` |
| File | [scripts/t1_tf_publisher.py](./scripts/t1_tf_publisher.py) |

This node is a transformation broadcaster. It basically publishes messages of type `tf2_msgs/TFMessage` on the topic `/tf` (which transmits all transformations in the system). The `tf2` framework is a wrapper to make handling transformation efficient.

#### Building

In `CMakeLists.txt` add the following

1. In the `find_package` function, add `tf2` and `tf2_ros`
2. In the `catkin_package` function under `CATKIN_DEPENDS`, add `tf2` and `tf2_ros`
3. Add the following to the `catkin_install_python` function

    ```txt
    scripts/t1_tf_publisher.py
    ```

After that, run `catkin_make` in the workspace directory

#### Running

This node is a part of tutorial 1. However, if you want to run this node in isolation, try running these commands and observe the output

Run `roscore`, then start RViz with the [configuration](#laserscan-and-tf-for-tutorial-1) for this tutorial. Then start this node by running

```bash
rosrun basic_robotics t1_tf_publisher.py
```

The output on RViz must appear to be somewhat like this

![Output of TF on RViz](./media/pic5.png)

After running this, you can use `rqt_tf_tree` to inspect the transformation tree at any instant. Run it using

```bash
rosrun rqt_tf_tree rqt_tf_tree
```

The output must be something show a transformation tree somewhat like this

![TF observed in rqt_tf_tree](./media/pic6.png)

### Marker and Static TF Publisher (Python) for Tutorial 1

| Field | Value |
| :---- | :---- |
| Name | `t1_py_markers_publisher` |
| File | [scripts/t1_markers_publisher.py](./scripts/t1_markers_publisher.py) |

This node publishes static transformations on the `/tf_static` topic. It also publishes a sphere marker. This node is included in the [launch file](#launch-python-for-tutorial-1) for [tutorial 1](#tutorial-1-visualizing-data-in-rviz).

#### Building and Running

In `CMakeLists.txt` add the following

1. In the `find_package` function, add dependency on `geometry_msgs` and `visualization_msgs`. Add them also to the `CATKIN_DEPENDS` list in `catkin_package` function.
2. Add `scripts/t1_markers_publisher.py` to `PROGRAMS` in the `catkin_install_python` function.

In the `package.xml` file, add `<build_depend>`, `<exec_depend>` and `<build_export_depend>` tags for `geometry_msgs` and `visualization_msgs` package (dependencies).

Build the package using `catkin_make` in the workspace directory. This node is included in the [launch file](#launch-python-for-tutorial-1) for [tutorial 1](#tutorial-1-visualizing-data-in-rviz).

### Robot Visualization (Python) for Tutorial 4

| Filed | Value |
| :--- | :---- |
| Name | `robot_joint_translator` |
| File | [scripts/t4_robot_joints.py](./scripts/t4_robot_joints.py) |

Gazebo publishes information about links in the simulation on topic `/gazebo/link_states` (information on models is published on `/gazebo/model_states`). Link transformations are inferred from these. The position of the robot is derived from `/tf` between `dummy` and `odom` frames. The original configurations of the wheels are passed to the node. The node calculates the change in orientation of the wheels and publishes the `sensor_msgs/JointState` message to the `joint_state_publisher`.

In the `CMakeLists.txt` file, in `catkin_install_python` function, add `scripts/t4_robot_joints.py` line (for installing the script).

## Configuration files

### Gazebo to Robot configurations for Tutorial 4

| Field | Value |
| :---- | :---- |
| Name | `t4_gz_robot_lframes.yaml` |
| File | [configs/t4_gz_robot_lframes.yaml](./configs/t4_gz_robot_lframes.yaml) |

Configurations for the `joint_state_publisher` and the [t4_robot_joints.py](#robot-visualization-python-for-tutorial-4) node. Specifies the topic where JointStates will be delivered, zero positions and gazebo link names for joints to be translated.

### Configurations for Tutorial 5

| Field | Value |
| :---- | :---- |
| Name | `t5_sim_configs.yaml` |
| File | [configs/t5_sim_configs.yaml](./configs/t5_sim_configs.yaml) |

Configurations for the `joint_state_publisher` node for tutorial 5. It contains the `sources_list` for the robot's joint feedback (through the plugin).

## RViz configuration files

### LaserScan and TF for Tutorial 1

| Field | Value |
| :---- | :---- |
| Name | `LaserScan_TF_T1.rviz` |
| File | [rviz/LaserScan_TF_T1.rviz](./rviz/LaserScan_TF_T1.rviz) |

This file is made for Tutorial 1. It is to view a `LaserScan` and a `TF`. Here's how to create it

1. Run `roscore` and `rviz`.
2. Add an `Axes` on the frame. This will serve as a reference frame (to know where the X, Y and Z of the world frame are). Customize it to your liking.
3. Add a `LaserScan`. This will be used to display a 2D laser scan in the frame. For now, let the topic be empty.
4. Add a `TF`. This will enable us to visualize the transformation tree on a topic called `/tf`.
5. Add a `Marker` and set the topic to `/visualization_marker` for source.
6. Save the configuration in the folder `rviz` inside the workspace (the file is linked above).
7. Close rviz and try running the following commands to test if everything is working

    ```bash
    roscd basic_robotics
    rviz -d ./rviz/LaserScan_TF_T1.rviz
    ```

    This would open `rviz` with the configuration that we saved. To know more about `-d` and other options, run `rviz --help`.

Hereon, only the things added will be briefly mentioned. This file may be modified throughout the tutorial.

### RobotDescription and TF for Tutorial 2

| Field | Value |
| :---- | :---- |
| Name | `RobotViz_T2.rviz` |
| File | [rviz/RobotViz_T2.rviz](./rviz/RobotViz_T2.rviz) |

This file is made for Tutorial 2. It is to view a `RobotDescription` (a robot model) and the transformations happening in real time (visualization). Here's how to create it.

1. Launch `rviz`
2. Add the following display types
    1. `Axes` for the global frame
    2. `TF` for transformations
    3. `RobotDescription` for visualizing a robot made using URDF
3. Change the `Fixed Frame` to `world`
4. Save the configuration in the folder `rviz` inside the workspace (the file is linked above).

This file may be modified throughout the tutorial.

### RobotDescription and TF for Tutorial 3

| Field | Value |
| :---- | :---- |
| Name | `RobotViz_T3.rviz` |
| File | [rviz/RobotViz_T3.rviz](./rviz/RobotViz_T3.rviz) |

This file follows the same procedure as that for [tutorial 2](#robotdescription-and-tf-for-tutorial-2).

### Robot visualization with sensor and TF for Tutorial 4

| Field | Value |
| :---- | :---- |
| Name | `RobotViz_T4.rviz` |
| File | [rviz/RobotViz_T4.rviz](./rviz/RobotViz_T4.rviz) |

This file is to visualize lidar data (LaserScan), camera image, transformations (both odometry and robot, separately) and robot description in RViz.

### Robot visualization for Tutorial 5

| Field | Value |
| :---- | :---- |
| Name | `Robot_Viz_T5.rviz` |
| File | [rviz/Robot_Viz_T5.rviz](./rviz/Robot_Viz_T5.rviz) |

This file is to visualize the single link robot and is created to visualize using the `joint_state_publisher_gui` or the `joint_state_publisher` with `source_list` for getting joint states.

## URDF Files

These are files created for robot description. The file format stands for *Unified Robot Description Format* (URDF). It provides creation of robot models in XML type files, having hierarchies of their own.

### Single block for Tutorial 2

| Field | Value |
| :---- | :---- |
| Name | `single_block_t2.urdf` |
| File | [urdf/single_block_t2.urdf](./urdf/single_block_t2.urdf) |

This is a file to introduce the concept of URDF and show how a rudimentary URDF file (with a robot) can be visualized in RViz.

### Two Blocks for Tutorial 2

| Field | Value |
| :---- | :---- |
| Name | `two_blocks_t2.urdf` |
| File | [urdf/two_blocks_t2.urdf](./urdf/two_blocks_t2.urdf) |

This is a file containing two blocks connected using a revolute joint. This is a part of the tutorial.

#### Checking URDFs

To check if a URDF file can be parsed correctly, there are several tools available. First, install `liburdfdom`. On systems with `apt`, the instruction is

```bash
sudo apt install liburdfdom-tools
```

This will install tools for inspecting URDFs. To inspect this file, run the following commands

```bash
roscd basic_robotics/urdf/
check_urdf ./two_blocks_t2.urdf
```

This will produce an output like following

![Check URDF output](./media/pic10.png)

## XACRO Files

The file format stands for *XML Macro* (XACRO). It is used to shorten and modularize big URDF files.

### Simple Four Wheel Bot for Tutorial 3

| Field | Value |
| :---- | :---- |
| Final URDF generated | [simple_fwb.urdf](./urdf/simple_fwb.urdf) |
| Main XACRO file | [urdf/simple_fwb.xacro](./urdf/simple_fwb.xacro) |
| Included XACRO files | [fwb_parameters.xacro](./urdf/fwb_parameters.xacro), [fwb_macros.xacro](./urdf/fwb_macros.xacro) |

A four wheel robot for [tutorial 3](#tutorial-3-gazebo-and-a-four-wheel-robot).The file descriptions are as follows

| File Name | Description |
| :--- | :--- |
| [urdf/fwb_parameters.xacro](./urdf/fwb_parameters.xacro) | Parameters for the four wheel robot |
| [urdf/fwb_macros.xacro](./urdf/fwb_macros.xacro) | Macros (functions that can be substituted when called) for the four wheel robot |
| [urdf/simple_fwb.xacro](./urdf/simple_fwb.xacro) | A simple version of the four wheel robot as a XACRO file |
| [urdf/simple_fwb.urdf](./urdf/simple_fwb.urdf) | URDF generated from the XACRO file `simple_fwb.xacro` |

To generate the `URDF` file from the `XACRO` file, use the following command

```bash
roscd basic_robotics/urdf/
xacro simple_fwb.xacro > simple_fwb.urdf
```

The `xacro` command generates a URDF using a XACRO file. It also includes values from the `<xacro:include ...>` in the given XACRO file. You can verify that URDF using

```bash
check_urdf simple_fwb.urdf
```

### Four Wheel Bot for Gazebo for Tutorial 3

| Field | Value |
| :---- | :---- |
| Main XACRO file | [fwb_gazebo_t3.xacro](./urdf/fwb_gazebo_t3.xacro) |
| Included XACRO files | [fwb_parameters.xacro](./urdf/fwb_parameters.xacro), [fwb_macros.xacro](./urdf/fwb_macros.xacro), [fwb_t3.gazebo](./urdf/fwb_t3.gazebo) |

A four wheel robot in Gazebo for [Tutorial 3](#tutorial-3-gazebo-and-a-four-wheel-robot). This is a more sophisticated version of the [simple_fwb.xacro](#simple-four-wheel-bot-for-tutorial-3) file above. The file descriptions are much the same, here are the additional files

| File Name | Description |
| :--- | :--- |
| [urdf/fwb_t3.gazebo](./urdf/fwb_t3.gazebo) | A Gazebo file containing `<gazebo>` for links and joints. Contains the plugin for Skid Steer Drive |

To generate and verify the `URDF` from the `XACRO` file, use the following commands

```bash
xacro fwb_gazebo_t3.xacro > fwb_gazebo_t3.urdf
gz sdf -p fwb_gazebo_t3.urdf
```

### Four Wheel Bot for Gazebo for Tutorial 4

| Field | Value |
| :---- | :---- |
| Main XACRO File | [urdf/fwb_gazebo_t4.xacro](./urdf/fwb_gazebo_t4.xacro) |
| Included files | [urdf/fwb_parameters_t4.xacro](./urdf/fwb_parameters_t4.xacro), [urdf/fwb_macros.xacro](./urdf/fwb_macros.xacro), [urdf/fwb_t4_general.gazebo](./urdf/fwb_t4_general.gazebo), [urdf/fwb_camera_t4.xacro](./urdf/fwb_camera_t4.xacro), [urdf/fwb_lidar_t4.xacro](./urdf/fwb_lidar_t4.xacro) |

A four wheel robot with a camera and a LiDAR (LaserScanner) sensor mounted. This mostly is derived from [fwb_gazebo_t3.xacro](#four-wheel-bot-for-gazebo-for-tutorial-3), but with additional `<plugin>` and `<sensor>` tags. There are additional parameters for the camera and laser scanner.The robot also starts with a dummy link for KDL to parse (`robot_state_publisher` needs it). Because of this, the `<plugin>` for SkidSteerDrive has `dummy` as the `<robotBaseFrame>` (first link of the robot is the dummy with no inertia).

### Single link robot for Tutorial 5

| Field | Value |
| :---- | :---- |
| Main XACRO File | [urdf/slbot_t5.urdf](./urdf/slbot_t5.urdf) |
| Included files | [slbot_t5_macros.xacro](./urdf/slbot_t5_macros.xacro), [slbot_t5.gazebo](./urdf/slbot_t5.gazebo) |

A single link robot that includes the plugin which is custom made for [tutorial 5](#tutorial-5-simple-1r-robot). The `<plugin>` is mapped to the custom plugin created. The macro also shows how to import STL meshes into your robot model.

## Gazebo World Files

Files made for Gazebo worlds in this package.

### Four Wheel Bot World for Tutorial 3

| Field | Value |
| :---- | :---- |
| Final World file | [world/t3_fwb_world.world](./world/t3_fwb_world.world) |

A world to test a four wheel robot. To create this file, run the following commands

```bash
roscore
rosrun gazebo_ros gazebo t3_fwb_world.world
```

This will open the gazebo simulator shown below

![Gazebo simulator](./media/pic16.png)

To create this file, the following procedure is followed

1. Add buildings for boundaries. Go to `Edit > Building Editor` for adding a layout

    ![Building layout for T3](./media/pic17.png)

    This lets you add walls to the layout above and create structures. You can double click on  line to edit its features. The end result must look like this (you could add some more features)

    ![Building floor for T3](./media/pic18.png)

    Exit the building editor using `File > Exit Building Editor`  and save it as `building_floor` in the `world/models` folder. After saving, a directory with the same name shows up containing the sdf files.

2. Add models to the editor. Go to `Edit > Model Editor` for adding model parts. Add models, double click on them to inspect

    ![Models on the building floor - T3](./media/pic19.png)

    Save this as `floor_model` in the `world/models` folder. SDF files are saved in this folder.

The model in the end looks like this (saved as `t3_fwb_world.world` in the `world` folder)

![T3 Part 1 Gazebo world](./media/pic20.png)

## Reference

- [RViz on roswiki](https://wiki.ros.org/rviz)
    - [User Guide](https://wiki.ros.org/rviz/UserGuide)
    - Built in [Display Types](https://wiki.ros.org/rviz/DisplayTypes)
- [RViz introduction YouTube](https://www.youtube.com/watch?v=i--Sd4xH9ZE&feature=emb_logo)
- [URDF on roswiki](https://wiki.ros.org/urdf)
    - [XACRO on roswiki](https://wiki.ros.org/xacro)
- [ROS Plugins for Gazebo](http://gazebosim.org/tutorials?tut=ros_plugins&cat=connect_ros)
    - ROS agnostic introduction to [plugins](http://gazebosim.org/tutorials?cat=write_plugin)
