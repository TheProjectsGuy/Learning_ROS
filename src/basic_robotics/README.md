# Basic Robotics

Basic Robotics related software provided in ROS. This package includes a brief overview of facilities like visualization, robotic simulation and sensors that ROS provides. Essential softwares like `RViz`, `Gazebo` and frameworks like `MoveIt!` are explored here.

## Table of contents

- [Basic Robotics](#basic-robotics)
    - [Table of contents](#table-of-contents)
    - [Creating this package](#creating-this-package)
    - [Foreword](#foreword)
    - [Tutorials](#tutorials)
        - [Getting Started with RViz](#getting-started-with-rviz)
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

Short tutorials included in this package made to cover essential concepts

### Getting Started with RViz

RViz is essentially a node that was created to visualize different aspects of robotics. Things like visualizing sensor data (like a Lidar scan, point cloud, path, and even cameral feeds), robot data (like robot models and frame transformations).

## Reference

- [RViz on roswiki](https://wiki.ros.org/rviz)
    - [User Guide](https://wiki.ros.org/rviz/UserGuide)
- [RViz introduction YouTube](https://www.youtube.com/watch?v=i--Sd4xH9ZE&feature=emb_logo)
