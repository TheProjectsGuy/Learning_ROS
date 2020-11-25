# Tutorial 1: Creating Workspace and Packages

This tutorial explains how to create your workspace and create packages inside a workspace.

## Table of contents

- [Tutorial 1: Creating Workspace and Packages](#tutorial-1-creating-workspace-and-packages)
    - [Table of contents](#table-of-contents)
    - [Foreword](#foreword)
    - [Workspace](#workspace)
        - [After build](#after-build)
    - [Packages](#packages)
        - [Building](#building)
        - [Tools](#tools)
            - [rospack](#rospack)
            - [rosls and roscd](#rosls-and-roscd)
    - [Conclusion](#conclusion)
    - [Reference](#reference)

## Foreword

Before getting started with this, it is assumed that you have a working ROS installation and have gone through the following files

- [I0_Basic_Terminology.md](./I0_Basic_Terminology.md): Basic terminology in ROS
- [T0_GS_Turtlesim.md](./T0_GS_Turtlesim.md): A codeless tutorial using TurtleSim

You may check if you have correctly sourced the ROS installation using

```bash
echo $ROS_DISTRO
```

And if everything is right, you must see `noetic` (or whatever distro you installed).

## Workspace

As explained earlier, a workspace is a directory on your system which contains all the ROS related, user developed content. This repository is created as a workspace on a local machine and pushed to the cloud.

> More about workspaces [here](http://wiki.ros.org/catkin/workspaces)

To create your own workspace, follow the following instructions

1. Create a folder and a `src` folder inside it

    This can be done using the `mkdir` command

    ```bash
    mkdir -p ~/ros_workspaces/learning_ws/src
    ```

    I prefer making a parent folder which contains different workspaces (sometimes I feel the need to work on completely different projects in ROS which need to be separate). The `ros_workspaces` folder acts as this directory. In roswiki tutorials, you may find that instead of `ros_workspaces/learning_ws` the workspace folder is mentioned as `catkin_ws`. You can use whatever name you want to, just make sure you remember it.
2. Go to your workspace folder

    For us, the workspace folder is `ros_workspaces/learning_ws`. It will be whatever you chose in step 1. You need to `cd` into it.

    ```bash
    cd ~/ros_workspaces/learning_ws
    ```

    For the rest of this repository, it is assumed that unless mentioned otherwise, you're running all commands from this folder.
3. Build your workspace for the first time

    After navigating into the workspace folder, run the `catkin_make` command

    ```bash
    catkin_make
    ```

    This command will build all the packages inside an existing workspace. However, if an empty workspace is used, it will create the files necessary for the workspace.

> roswiki on _Creating a ROS workspace using Catkin_ [here](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace)

### After build

After the first build, a few things can be noted

1. In the `src` folder inside your workspace, a file called `CMakeLists.txt` would have appeared. This file is actually a soft copy of a template build file. Under **no** ordinary circumstances, you are to modify this file. All the build configurations that packages require are done through `CMakeLists.txt` file inside the package folders and not this one.
2. Two new folders named `build` and `devel` would have formed.
    1. The `build` folder is used by CMake and catkin to keep their cache information. This folder aids in building everything in the package.
        > More about the **build** space [here](http://wiki.ros.org/catkin/workspaces#Build_Space)
    2. The `devel` folder is used to store built targets (executables, library headers, etc.). This is useful for ROS to locate them.
        > More about **devel** space [here](http://wiki.ros.org/catkin/workspaces#Development_.28Devel.29_Space)
    - In case you want to rebuild everything from ground up, you can delete the `build` and `devel` folders (you may use `rm -r`) and then again run `catkin_make` in the workspace folder.
3. **Source your workspace**: Inside the `devel` folder, there must be files named `setup.*`. In order for ROS to know where the workspace is, you need to source the workspace. Without this, packages inside your workspace shall not be discoverable.

    To confirm this, you may first run

    ```bash
    echo $ROS_PACKAGE_PATH
    ```

    If you do not have any other workspace sourced, you must see only `/opt/ros/noetic/share` in the output (installation path). Let us source the workspace that we just created

    ```bash
    source ~/ros_workspaces/learning_ws/devel/setup.bash
    ```

    And now run

    ```bash
    echo $ROS_PACKAGE_PATH
    ```

    Now, in the output, we can also see the `src` folder of the workspace that we created. If you use the default `bash` shell, then you must source `setup.bash`. If you're using another shell, say `zsh`, sourcing `setup.zsh` may be a better idea.

    - The above `source` command will have to be executed every time you open a new terminal. Otherwise, the `$ROS_PACKAGE_PATH` variable would not be updated. Therefore, it is a good idea to have the source line in the `~/.bashrc` file, so that every time you open a new terminal, the workspace is sourced also (similar to what you may have done by sourcing the ROS installation file when installing ROS).

        Run the following command

        ```bash
        echo "source ~/ros_workspaces/learning_ws/devel/setup.bash" >> ~/.bashrc
        ```

        Note that you may use `~/.bashrc` if you use bash. If you're using, say zsh, you might want to use the command

        ```bash
        echo "source ~/ros_workspaces/learning_ws/devel/setup.zsh" >> ~/.zshrc
        ```

## Packages

A package is used to contain all the things made for a particular purpose inside a single folder. These folders are stored inside the `src` folder of the workspace.

> More about packages [here](http://wiki.ros.org/Packages)

To create your own package, you must first have the following information in mind

- **Package name**: An appropriate name for your package. For the purpose of this tutorial, let it be `cpp_basic_nodes`.
- **Dependencies**: They are other packages and libraries on which your package depends to build and run everything inside it. You can even add or remove them _after_ creating the package. However, it's better to have the known ones set up with creation. For this tutorial, we'll use the following dependencies
    - `roscpp`: Since this is a C++ package, it uses the C++ client library.

To create a package, follow these steps

1. Make sure that you are in the `src` folder of your workspace and that the workspace and ROS is properly sourced.

    ```bash
    cd ~/ros_workspaces/learning_ws/src
    ```

2. Run the following command to create a package using `catkin_create_pkg`

    ```bash
    catkin_create_pkg cpp_basic_nodes roscpp
    ```

This must have create a folder named `cpp_basic_nodes` and the folder must have

- `CMakeLists.txt` file: This is to manage how things in the package are built
    > More about the CMakeLists.txt file [here](http://wiki.ros.org/catkin/CMakeLists.txt)
- `package.xml` file: For managing the manifesto and information about the package
    > More about the package.xml file [here](http://wiki.ros.org/catkin/package.xml)

### Building

After the package is created, you might want to build it (even if its empty, just to see if everything is fine). To do that, first `cd` into the workspace folder

```bash
cd ~/ros_workspaces/learning_ws/
```

Now run the following

```bash
catkin_make
```

You must see a message in the output mentioning that your package `cpp_basic_nodes` was found (under traversing N packages in topological order).

To selectively build packages, you may use the `--pkg` tag with `catkin_make`.

### Tools

Some tools that are useful in handling packages

#### rospack

The `rospack` tool can be used to get information about packages. For example, to get the basic dependencies of a package, you may run

```bash
rospack depends1 cpp_basic_nodes
```

To get all dependencies, you may use

```bash
rospack depends cpp_basic_nodes
```

You can use `rospack help` to get more information about the tool.

> More about the **rospack** tool [here](http://wiki.ros.org/rospack)

#### rosls and roscd

The `rosls` tool is used to list the contents of a package (irrespective of the directory you're in).

```bash
rosls cpp_basic_nodes
```

Will list the contents of the `cpp_basic_nodes` package.

The `roscd` tool is used to `cd` into a package directory

```bash
roscd cpp_basic_nodes
```

Will get you into the directory `~/ros_workspaces/learning_ws/src/cpp_basic_nodes` (if you did everything according to this tutorial)

## Conclusion

In this tutorial, you learned how to make your own workspace and create packages inside the workspace. You also learned how to build the packages in a workspace.

## Reference

- Tutorials on roswiki
    - Creating a workspace [here](https://wiki.ros.org/catkin/Tutorials/create_a_workspace)
    - Creating a package [here](https://wiki.ros.org/catkin/Tutorials/CreatingPackage)
    - Navigating the ROS filesystem [here](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem)
