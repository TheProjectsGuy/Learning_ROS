# Learning ROS

![ROS Version: Noetic][ros-support-badge]
![OS: Ubuntu 20.04][os-support-badge]

This repository is for learning Robot Operating System (ROS) through a hands on approach.

## Table of contents

- [Learning ROS](#learning-ros)
    - [Table of contents](#table-of-contents)
    - [Foreword](#foreword)
        - [Prerequisites](#prerequisites)
        - [About the contents](#about-the-contents)
    - [Contents](#contents)
        - [Folders](#folders)
    - [References and Credits](#references-and-credits)

## Foreword

Things to keep in mind before getting started

### Prerequisites

Before proceeding further, the following is needed:

- It is assumed that you have basic knowledge of shell scripting and linux concepts. Skim through [this][terminal-beginners] anyways.
- A ROS installation: This repository uses ROS Noetic, on Ubuntu 20.04 (Focal). Installation procedure can be found [here][roswiki-default-install].
    > ROS installation page [here][roswiki-install]

Knowledge about the following is a plus:

- [Git](https://git-scm.com/) version control system: This may be useful in traversing this repository.

### About the contents

The contents of this repository are organized as a *tree*. If one traverses in the mentioned order of links, then they shall cover the entire repository well (which is suggested for beginners). However, if one is looking for something specific, they can straightaway search and explore randomly.

This repository is actually a ROS Workspace which you can directly clone and build (more about these terms later). However, for beginners, it may be better to code and build everything on their own after understanding the material (rather than cloning, building and then running)

The following editors and IDEs are used extensively for the purpose of this repository:

- [VSCode](https://code.visualstudio.com/), you may want to bookmark the [keyboard shortcuts page](https://code.visualstudio.com/shortcuts/keyboard-shortcuts-windows.pdf). Check [this](http://wiki.ros.org/IDEs#Visual_Studio_Code_.28VSCode.29) out for using it with ROS.
- [KDevelop](https://www.kdevelop.org/) IDE for C++ programming. Check [this](http://wiki.ros.org/IDEs#KDevelop) out for using it with ROS.
- [PyCharm](https://www.jetbrains.com/pycharm/) IDE for Python Programming (suggested to install using [toolbox app](https://www.jetbrains.com/toolbox-app/)). Check [this](http://wiki.ros.org/IDEs#PyCharm_.28community_edition.29) out for using it with ROS.

> More about IDEs suggested for ROS [here](http://wiki.ros.org/IDEs)

## Contents

Contents in the order of traversal

| S. No. | Name | Description |
| :---- | :---- | :------ |
| 1 | [I0_Basic_Terminology.md](./tutorials/I0_Basic_Terminology.md) | **Basic Terminology**: Description about various terms and concepts used in ROS |
| 2 | [T0_GS_Turtlesim.md](./tutorials/T0_GS_Turtlesim.md) | **Understanding ROS concepts without code**: Basic ROS concepts demonstrated using `turtlesim` |
| 3 | [T1_Workspace_Package.md](./tutorials/T1_Workspace_Package.md) | **Workspace and Package**: Create your own workspace and packages |
| 4 | [src/README.md](./src/README.md) | **Main Packages**: Packages made for learning ROS |

### Folders

Description of folders present

| S. No. | Name | Description |
| :--- | :---- | :----- |
| 1 | [src](./src/) | Packages for the workspace. They contain the code |
| 2 | [tutorials](./tutorials/) | Contains tutorial and some information files |

## References and Credits

[![Developer: TheProjectsGuy][dev-shield]][dev-link]

[ros-support-badge]: https://img.shields.io/badge/ROS%20Version-ROS%20Noetic%20Ninjemys-3ad12c
[os-support-badge]: https://img.shields.io/badge/OS-Ubuntu%20Focal-orange
[terminal-beginners]:https://ubuntu.com/tutorials/command-line-for-beginners#1-overview
[roswiki-install]: http://wiki.ros.org/ROS/Installation
[roswiki-default-install]: http://wiki.ros.org/noetic/Installation/Ubuntu
[dev-shield]: https://img.shields.io/badge/Developer-TheProjectsGuy-blue
[dev-link]: https://github.com/TheProjectsGuy/
