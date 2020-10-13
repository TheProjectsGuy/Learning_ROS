# Learning ROS

![OS: Ubuntu 20.04][os-tag] ![ROS Version: Neotic Ninjemys][rosversion-tag]

This repository strives to serve as a starting point towards the journey of learning ROS. This repository also strives to be a reference point for ROS resources.

## Table of contents

- [Learning ROS](#learning-ros)
    - [Table of contents](#table-of-contents)
    - [Prerequisites](#prerequisites)
        - [Repository structure](#repository-structure)
    - [Content](#content)
        - [Setting up everything](#setting-up-everything)

## Prerequisites

Before getting started, it is important to get a few things set up. Make sure you have:

1. A computer running Ubuntu. Though other operating systems also support ROS, this repository primarily focuses on **Ubuntu 20.04 (focal)**. Download it from [here][ubuntu-download].
    > If you intend to dual boot, you can check [this][ubuntu-dualboot] out.
2. A successful installation of ROS. This repository focuses on **ROS Noetic Ninjemys**.
    > Read [this][ros-installation] for knowing how to install ROS on Ubuntu.
3. A useable editor. This repository uses **VSCode** (extensions and intellisense is great). Get it from [here][vscode-website].
    > Useful VSCode extensions: [C++][vscode-ext-cpp], [Python][vscode-ext-python] and [ROS][vscode-ext-ros].

### Repository structure

This repository consists of various packages that one needs to get started. This entire repository resides within a single workspace.

> _Package_: A package is a folder that contains the source material for accomplishing a particular task.

> _Workspace_: A workspace consists of source code for packages, files built using those source code and many other items ROS related. It's everything in a single folder.

The following points can be kept in mind:

- This entire repository is made as a workspace. So all the code files are divided into packages. It's suggested that you try writing your own code and use these files just for reference. However, you can directly copy packages (as folders) from this repository as well.

## Content

### Setting up everything

1. The workspace is created by first creating a folder (with an `src` folder in it)

    ```bash
    mkdir -p ~/ros_workspaces/learning_ws/src/
    ```

    The folder `~/ros_workspaces/learning_ws` will be called our **workspace directory** (you can use any folder).

[![Developer: TheProjectsGuy][dev-shield]][dev-link]

[dev-shield]: https://img.shields.io/badge/Developer-TheProjectsGuy-00bbd8
[dev-link]: https://github.com/TheProjectsGuy
[os-tag]: https://img.shields.io/badge/Operating%20System-Ubuntu%2020.04%20(focal)-orange
[rosversion-tag]: https://img.shields.io/badge/ROS%20Release-Noetic%20Ninjemys-blue
[ros-installation]: http://wiki.ros.org/noetic/Installation
[ubuntu-download]: https://ubuntu.com/download/desktop
[ubuntu-dualboot]: https://www.tecmint.com/install-ubuntu-alongside-with-windows-dual-boot/
[vscode-website]: https://code.visualstudio.com/
[vscode-ext-cpp]: https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools
[vscode-ext-python]: https://marketplace.visualstudio.com/items?itemName=ms-python.python
[vscode-ext-ros]: https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros
