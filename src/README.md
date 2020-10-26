# Source code for Learning_ROS Packages

A ROS workspace contains several packages. The source code of all those packages is stored inside the `src` folder in the workspace directory.

## Table of contents

- [Source code for Learning_ROS Packages](#source-code-for-learning_ros-packages)
    - [Table of contents](#table-of-contents)
    - [Prerequisites](#prerequisites)
        - [Sourcing](#sourcing)
        - [Creating a package](#creating-a-package)
    - [Packages](#packages)
    - [Contents](#contents)

## Prerequisites

### Sourcing

In order to run a package, make sure that

1. ROS is sourced
2. Workspace is sourced

Both can be verified by running

```bash
echo $ROS_PACKAGE_PATH
```

and inspecting the output. If it does not contain your package path or the ROS path, then try running the following

```bash
source /opt/ros/noetic/setup.bash
source ~/ros_workspaces/learning_ws/devel/setup.bash
```

which will source the installation and the workspace directory. You're suggested to put these lines in the `~/.bashrc` file (if you want new terminals to automatically source them).

### Creating a package

To create a package (say `cpp_basic_nodes`), do the following:

1. Go to the workspace directory

    ```bash
    cd /home/avneesh/ros_workspaces/learning_ws/src
    ```

2. Run `catkin_create_pkg` command to create a package. You could pass the dependencies now, or add them later.

    ```bash
    catkin_create_pkg cpp_basic_nodes std_msgs roscpp
    ```

    The package name here is `cpp_basic_nodes` and the dependencies are `roscpp` (ROS for C++) and `std_msgs` (standard messages).

3. After then, you could build the workspace with that package. This would build the package.

    ```bash
    catkin_make
    ```

    You could also use the `--pkg PKG [PKG ...]` argument to build specific packages only. More about options can be found by running `catkin_make --help`. Note that after the build, you can see a folder representing the cmake files used for the package in the `devel/share` folder of the workspace.

## Packages

Information about all the packages in this folder. The packages are divided into sections based on the difficulty level. Click on their name to go to their folder (which contains a README for the specific package).

## Contents

The contents of this folder are as follows
