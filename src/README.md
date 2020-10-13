# Source code for Learning_ROS Packages

A ROS workspace contains several packages. The source code of all those packages is stored inside the `src` folder in the workspace directory.

## Table of contents

- [Source code for Learning_ROS Packages](#source-code-for-learning_ros-packages)
    - [Table of contents](#table-of-contents)
    - [Prerequisites](#prerequisites)
        - [Naming convention](#naming-convention)
    - [Contents](#contents)

## Prerequisites

In order to run a package, make sure that

1. ROS is sourced
2. Workspace is sourced

Both can be verified by running

```bash
echo $ROS_PACKAGE_PATH
```

and inspecting the output.

### Naming convention

There are many packages in this folder for different purposes. They are named using a convention as described below:

- There are no spaces in the name, all separators are `_` (underscores).
- The first word for a programming package (a package that has primarily code in it) is either `cpp` (for packages written in C++) or `py` (for packages written in python). This repository uses primarily C++ or Python for programming.

## Contents

The contents of this folder are as follows
