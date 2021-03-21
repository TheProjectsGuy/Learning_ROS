# Shared Libraries

Creating libraries in ROS packages that can be used in other libraries. This package includes a python library and a C++ library that can be accessed by other packages. The methodology of building them is also presented.

## Table of contents

- [Shared Libraries](#shared-libraries)
    - [Table of contents](#table-of-contents)
    - [Creating this package](#creating-this-package)
    - [Foreword](#foreword)
    - [Contents](#contents)
        - [Python Libraries](#python-libraries)
            - [Shared Python Library](#shared-python-library)
                - [Building](#building)
                - [Testing](#testing)

## Creating this package

This package was created using the following commands

```bash
cd ~/ros_workspaces/learning_ws/src
catkin_create_pkg shared_libs roscpp rospy
```

## Foreword

This package consists of Python and C++ libraries

## Contents

### Python Libraries

The Python libraries are put in a folder named [py_modules](./py_modules/).

#### Shared Python Library

| Field | Value |
| :---- | :---- |
| Library Name | `shared_pylib` |
| Initialization File | [py_modules/shared_pylib/](./py_modules/shared_pylib/__init__.py) |

This is a simple Python module / package containing another package named `simple_math`. Explore the code before going forward and building it (building here means to make it accessible to use everywhere in the workspace).

##### Building

To build the python library, a [setup.py](./setup.py) file is used. The file ensures that the setup procedure knows where to find the libraries and know their names.

In the CMakeLists.txt file

1. Uncomment the `catkin_python_setup()` function (in the beginning). This is for catkin to recognize and build on the `setup.py` file in the package.

> More on the `catkin_python_setup` function [here](https://docs.ros.org/en/latest/api/catkin/html/user_guide/setup_dot_py.htmls)

After that, run `catkin_make` in the workspace. After a successful build, a folder `devel/lib/python3/dist-packages/shared_pylib` must have been created in the workspace. It would include an `__init__.py` file to effectively run the library in the `py_modules` folder.

##### Testing

A Python node was made to test the library made. Check out [scripts/basic_module.py](./scripts/basic_module.py) for that. Building this includes `catkin_install_python` function in CMakeLists.txt.
