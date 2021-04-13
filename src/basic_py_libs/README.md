# Python Libraries

Basic python libraries to demonstrate creating modules in ROS packages.

## Table of content

- [Python Libraries](#python-libraries)
    - [Table of content](#table-of-content)
    - [Creating this package](#creating-this-package)
    - [Foreword](#foreword)
    - [Contents](#contents)
    - [Modules](#modules)
        - [SimpleModule](#simplemodule)
            - [Building](#building)
        - [BasicMath](#basicmath)
            - [Building](#building-1)
    - [Nodes](#nodes)
        - [SimpleModuleNode](#simplemodulenode)
            - [Building and Running](#building-and-running)
        - [BasicMathNode](#basicmathnode)
            - [Building and Running](#building-and-running-1)
    - [Reference](#reference)

## Creating this package

This package was created using the following commands

```bash
cd ~/ros_workspaces/learning_ws/src
catkin_create_pkg basic_py_libs rospy
```

## Foreword

This package is to serve as a collection of libraries. These libraries could be used by nodes of this package or other packages. Traditionally, it is more efficient to package functionality as modules (or libraries) that you can include (or import) in your nodes.

## Contents

Suggested order of traversal for the items in this package (specially for beginners)

| S. No. | Name | Link | Description |
| :---- | :---: | :--- | :---- |
| 1 | Simple Module | [Modules > SimpleModule](#simplemodule) | A basic module for demonstrating modules |
| 2 | Node: Simple Module | [Nodes > SimpleModuleNode](#simplemodulenode) | A node to demonstrate using the `simple_module` library |
| 3 | Basic Math Module | [Modules > BasicMath](#basicmath) | A basic math module, created to demonstrate sub-modules |
| 4 | Node: Basic Math Module | [Node > BasicMathNode](#basicmathnode) | A node to demonstrate using the `basic_math` library |

## Modules

Modules defined in this package. As a convention, modules are put inside the `src` folder.

### SimpleModule

| Field | Value |
| :--- | :--- |
| Path | [src/simple_module](./src/simple_module/README.md) |

A module to only demonstrate how python modules can be created in packages.

#### Building

Python code usually do not require building, but to make your created modules importable to other packages, they must be made available in the `devel` folder  (in the workspace) and the variable `PYTHONPATH` must be managed. All this is handled using a build procedure. In brief, to expose your custom python module to other packages, do the following

1. Create a file called `setup.py` in your package's root folder. You can use the one [here](./setup.py) as a template (which is built upon the [reference](https://docs.ros.org/en/api/catkin/html/howto/format2/installing_python.html#modules)). Use the following instructions to create your own

    1. First import the basic modules that will be used to create an exportable package

        ```py
        from setuptools import setup
        from catkin_pkg.python_setup import generate_distutils_setup
        ```

    2. Call the `generate_distutils_setup` function to generate the setup dictionary (a dictionary that will later be passed as arguments to the `setup` function). Include a list of packages in the `packages` argument and their path in the `package_dir` argument

        ```py
        setup_args = generate_distutils_setup(
            packages=['simple_module'],
            package_dir={'': 'src'}
        )
        ```

    3. Call the `setup` function with arguments

        ```py
        setup(**setup_args)
        ```

2. In the `CMakeLists.txt` file, uncomment `catkin_python_setup` (it's after `find package` in the beginning, just before `Declare ROS messages, services and actions` header)

    ```txt
    catkin_python_setup()
    ```

    You could also add this immediately after (instead of un-commenting)

3. Run `catkin_make` in the workspace directory to build your workspace.

After a successful build, you must see a folder `devel/lib/python3/dist-packages/simple_module` in your workspace. This is the module that can be imported by nodes (it is a wrapper around your actual module) and the generation of this is what is implied by _installation_ of a Python library. 

A [sample node](#simplemodulenode) is written in this package. You may have to source your ROS workspace again, do it by running the following in the workspace directory

```bash
source ./devel/setup.bash
```

### BasicMath

| Field | Value |
| :--- | :--- |
| Path | [src/basic_math](./src/basic_math/README.md) |

A module made to demonstrate how submodules can be made and imported. The module's tree is as follows

```txt
basic_math
├── __init__.py
├── algebra
│   ├── __init__.py
│   └── real_number.py
└── imp_functions
    ├── __init__.py
    └── factorial.py

```

#### Building

Include the package name `basic_math` in the list of `packages` in `generate_distutils_setup` function of the `setup.py`. After this, the `setup.py` file must look like this

```py
# Create dictionary for setup using package.xml
setup_args = generate_distutils_setup(
    # A list of packages in the repository
    packages=[
            'simple_module', # In ./src/simple_module
            'basic_math',    # In ./src/basic_math (Note: You could include other modules)
        ],
    # Directory in which packages are located (usually `src`)
    package_dir={'': 'src'}
)
```

Then, run `catkin_make` in the workspace directory. Just like in the case of `simple_module` library, you must now see a `basic_math` module under `devel/lib/python3/dist-packages/` folder. This means that the library is successfully installed.

A [sample node](#basicmathnode) is written in this package. You may have to source the workspace after building this library.

## Nodes

Nodes created in this package. Most of them are for demonstrating uses of the modules in this package.

### SimpleModuleNode

| Field | Value |
| :--- | :--- |
| Name | `simple_module_node` |
| File | [scripts/simple_module_node.py](./scripts/simple_module_node.py) |

This node simply includes the functionality of the `simple_module` python [module](#simplemodule) in this package.

#### Building and Running

Add the function `catkin_install_python` in CMakeLists.txt (located under the `install` header).

```txt
catkin_install_python(PROGRAMS
  scripts/simple_module_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

Then, run `catkin_make` in the workspace directory. To run this node, ensure that `roscore` is running first and then run

```bash
rosrun basic_py_libs simple_module_node.py
```

### BasicMathNode

| Field | Value |
| :--- | :--- |
| Name | `basic_math_node` |
| File | [scripts/basic_math_node.py](./scripts/basic_math_node.py) |

This node simply demonstrates how to access functionality of the `basic_math` python module.

#### Building and Running

Add the script `script/basic_math_node.py` to `catkin_install_python` function in CMakeLists.txt and run `catkin_make` in the workspace directory. To run this node, first run `roscore` and then run

```bash
rosrun basic_py_libs basic_math_node.py
```

## Reference

- How to install Python Modules [through catkin](https://docs.ros.org/en/api/catkin/html/howto/format2/installing_python.html#modules)
