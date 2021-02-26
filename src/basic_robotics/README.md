# Basic Robotics

Basic Robotics related software provided in ROS. This package includes a brief overview of facilities like visualization, robotic simulation and sensors that ROS provides. Essential softwares like `RViz`, `Gazebo` and frameworks like `MoveIt!` are explored here.

## Table of contents

- [Basic Robotics](#basic-robotics)
    - [Table of contents](#table-of-contents)
    - [Creating this package](#creating-this-package)
    - [Foreword](#foreword)
    - [Tutorials](#tutorials)
        - [Tutorial 0: Getting Started with RViz](#tutorial-0-getting-started-with-rviz)
    - [RViz configuration files](#rviz-configuration-files)
        - [LaserScan and TF for Tutorial 1](#laserscan-and-tf-for-tutorial-1)
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
| 2 | [Visualizing data in RViz](#tutorial-1-visualizing-data-in-rviz) | Visualize `TF` and `LaserScan` using dummy publishers |

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
5. Save the configuration in the folder `rviz` inside the workspace (the file is linked above).
6. Close rviz and try running the following commands to test if everything is working

    ```bash
    roscd basic_robotics
    rviz -d ./rviz/LaserScan_TF_T1.rviz
    ```

    This would open `rviz` with the configuration that we saved. To know more about `-d` and other options, run `rviz --help`.

Hereon, only the things added will be briefly mentioned.

## Reference

- [RViz on roswiki](https://wiki.ros.org/rviz)
    - [User Guide](https://wiki.ros.org/rviz/UserGuide)
    - [Display Types](https://wiki.ros.org/rviz/DisplayTypes)
- [RViz introduction YouTube](https://www.youtube.com/watch?v=i--Sd4xH9ZE&feature=emb_logo)
