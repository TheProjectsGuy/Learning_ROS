# C++ Libraries

Basic C++ libraries to demonstrate creating C++ libraries in ROS packages.

## Table of contents

- [C++ Libraries](#c-libraries)
    - [Table of contents](#table-of-contents)
    - [Creating this package](#creating-this-package)
    - [Foreword](#foreword)
    - [Contents](#contents)
    - [Libraries](#libraries)
        - [SimpleLibrary](#simplelibrary)
            - [Building](#building)
        - [BasicMath](#basicmath)
            - [Building](#building-1)
    - [Nodes](#nodes)
        - [SimpleLibraryNode](#simplelibrarynode)
            - [Building and Running](#building-and-running)
        - [BasicMathNode](#basicmathnode)
            - [Building and Running](#building-and-running-1)
    - [Reference](#reference)

## Creating this package

This package is created using the following commands

```bash
cd ~/ros_workspaces/learning_ws/src
catkin_create_pkg basic_cpp_libs roscpp
```

## Foreword

This package is to serve as a collection of libraries. These libraries could be used by nodes of this package or other packages. Traditionally, it is more efficient to package functionality as modules (or libraries) that you can include (or import) in your nodes.

## Contents

Suggested order of traversal for the items in this package (specially for beginners)

| S. No. | Name | Link | Description |
| :---- | :---: | :--- | :---- |
| 1 | Simple Library | [Libraries > SimpleLibrary](#simplelibrary) | A simple C++ library for demonstration |
| 2 | Node for Simple Library | [Nodes > SimpleLibraryNode](#simplelibrarynode) | A node demonstrating the use of `simple_library` library |
| 3 | Basic Math library | [Libraries > BasicMath](#basicmath) | A library to demonstrate a more complex library in a ROS package |
| 4 | node for Math library | [Nodes > BasicMath](#basicmathnode) | A node demonstrating the use of `basic_math` library |

## Libraries

Libraries defined in this package. As a convention, libraries are put inside the `src` folder.

### SimpleLibrary

| Field | Value |
| :--- | :--- |
| Header file | [include/basic_cpp_libs/simple_library.h](./include/basic_cpp_libs/simple_library.h) |
| Source code | [src/simple_library.cpp](./src/simple_library.cpp) |

A library to only demonstrate how C++ libraries can be created in packages.

#### Building

C++ code requires building. These procedures can be found [easily](https://docs.ros.org/en/latest/api/catkin/html/howto/format2/building_libraries.html). In the `CMakeLists.txt` file, do the following

1. Make sure that `roscpp` is set up as a dependency (in both `CMakeLists.txt` and `package.xml` files).
2. In the `catkin_package` function (under `catkin specific configuration` header), modify it to the following

    ```makefile
    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp
        #  DEPENDS system_lib
    )
    ```

    The `INCLUDE_DIRS` is to include the folders containing header file code. The `LIBRARIES` is used to export a name for dependent packages. The `CATKIN_DEPENDS` is used to list the `catkin_packages` used.

3. In the `include_directories` function (under the `Build` header), modify it to the following

    ```makefile
    include_directories(
        include
        ${catkin_INCLUDE_DIRS}
    )
    ```

    List of directories that contains the header files

4. After that, add a `add_library` function for including source code file

    ```makefile
    add_library(${PROJECT_NAME}
        src/simple_library.cpp
    )
    ```

    List of `.cpp` source files for the library. Things are declared in the header files (listed in the `include_directories` function) and defined in these `.cpp` files (source files).

5. Link the libraries for this library using `target_link_libraries`, add the following at the appropriate place

    ```makefile
    target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
    ```

    This will allow catkin libraries to be linked to this library (which is named `${PROJECT_NAME}`)

6. Mark this library for installation (these will be placed under `Install` header). Add the following under `Mark libraries for installation`

    ```makefile
    install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    )
    ```

    - The `TARGETS` list out the library names that this package has
    - The `ARCHIVE DESTINATION` command is for the archiving folder, the `${CATKIN_PACKAGE_LIB_DESTINATION}` [variable](https://docs.ros.org/en/latest/api/catkin/html/user_guide/variables.html) is essentially set to `lib`
    - The `LIBRARY DESTINATION` command is for the destination to store the library file (that can be directly linked). This is also indirectly set to `lib` (through the variable), which is the folder in `devel` (inside the workspace) where the `.so` file will go.
    - The `RUNTIME DESTINATION` command is for the runtime binaries. This is set to `bin` through the variable.

7. Now that all configurations are done, we need to tell the final destination where the header files must be installed to. Add the following `install` function under the one described above

    ```makefile
    install(
        DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )
    ```

    This is finally done to expose the header files.

Now that everything is set, run `catkin_make` in the workspace directory. After a successful build, a file by the name of `libbasic_cpp_libs.so` must be available in the folder `devel/lib/` (in the workspace). This is the library file that can be linked to (for creating nodes).

A [node in this package](#simplelibrarynode) is made for demonstrating this library.

### BasicMath

| Field | Value |
| :--- | :--- |
| Main header file | [include/basic_math/basic_math.hpp](./include/basic_math/basic_math.hpp) |

A relatively more sophisticated library for demonstrating complex C++ libraries inside ROS packages.

The directory structure for header files is as follows

```txt
include
├── basic_cpp_libs
│   └── simple_library.h
└── basic_math
    ├── algebra
    │   ├── algebra.hpp
    │   ├── real_number.hpp
    │   └── real_number.ipp
    ├── basic_math.hpp
    └── imp_functions
        ├── factorial.hpp
        └── imp_functions.hpp
```

And the source files are as following (in the `src` folder)

```txt
src/basic_math
└── imp_functions
    └── factorial.cpp
```

#### Building

C++ Libraries in ROS packages has to be built. Traditionally, like the [simple_library](#simplelibrary), the headers are included in the `include/PACKAGE_NAME` folder. But the folder name and even the library name can be changed to anything and that is experimented here.

In the `CMakeLists.txt` file, do the following

1. In the `catkin_package` function (under `catkin specific configuration` header), add the library name to the `LIBRARIES` list. The function is modified to this

    ```makefile
    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_basic_math
        CATKIN_DEPENDS roscpp
        #  DEPENDS system_lib
    )
    ```

    Note that the `${PROJECT_NAME}` library name is taken by the [simple_library](#simplelibrary) of this package. This library will take the name `${PROJECT_NAME}_basic_math`. Other arguments are same as in case of `simple_library`

2. Each library requires a dedicated `add_library` function for pointing out the source code (definitions for all the things declared in the headers). For this one, the function may look like this

    ```makefile
    add_library(${PROJECT_NAME}_basic_math
        # Important functions (source code)
        src/basic_math/imp_functions/factorial.cpp
    )
    ```

    There can be many source files in this list (as are required for the entire library)

3. The library has to be linked to `${catkin_LIBRARIES}`, this is done using `target_link_libraries`. Add the following

    ```makefile
    target_link_libraries(${PROJECT_NAME}_basic_math ${catkin_LIBRARIES})
    ```

    This will link catkin libraries with this library

4. This library will be installed at the same space, so it will share the marked libraries as the `simple_library`. Modify the `install` function to mark libraries for installation to the following

    ```makefile
    install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_basic_math
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    )
    ```

    Note that this library `${PROJECT_NAME}_basic_math` is simply appended to the list of libraries under `TARGETS`. This is because this library shares the `ARCHIVE DESTINATION` and other settings with the `simple_library`. Most of the libraries belonging to a package share these settings.

5. Add an `install` to set the install destination for the C++ header files. As this library is not in a folder bearing the package name inside `include`, the `install` command is a little different from before

    ```makefile
    install(
        DIRECTORY include/basic_math
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/basic_math
    )
    ```

    Notice that the `DIRECTORY` for this library is set to `include/basic_math` which has the header files (unconventional, but can work). Similarly, the `DESTINATION` is set to `${CATKIN_GLOBAL_INCLUDE_DESTINATION}/basic_math` instead of `${CATKIN_PACKAGE_INCLUDE_DESTINATION}`. The difference is

    - `${CATKIN_PACKAGE_INCLUDE_DESTINATION}` is set to `${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}`, which becomes `include/basic_cpp_libs`. Therefore, the `#include` statements will start includes from that folder (that is, `#include<basic_cpp_libs/...`).
    - `${CATKIN_GLOBAL_INCLUDE_DESTINATION}` is set to `include`, therefore `${CATKIN_GLOBAL_INCLUDE_DESTINATION}/basic_math` becomes `include/basic_math`. The `#include` statements will start includes with `#include<basic_math/...`.

After this, run `catkin_make` in the workspace folder. After a successful build, a file named `libbasic_cpp_libs_basic_math.so` shall appear in the folder `devel/lib` for signifying this library. This file will be linked with executables.

A [node in this package](#basicmathnode) is made for demonstrating this library.

## Nodes

Nodes contained in this package. Most of them are for demonstrating uses of the libraries in this package.

### SimpleLibraryNode

| Field | Value |
| :--- | :--- |
| Name | `simple_library_node` |
| Code | [src/simple_library_node.cpp](./src/simple_library_node.cpp) |

A C++ node made to demonstrate using the `simple_library` [library](#simplelibrary) of this package.

#### Building and Running

To build and run this node, add the following lines to the `CMakeLists.txt` file

```makefile
add_executable(simple_library_node src/simple_library_node.cpp)
target_link_libraries(simple_library_node ${catkin_LIBRARIES} ${PROJECT_NAME})
```

Notice that the `target_link_libraries` for the `simple_library_node` executable links the `${PROJECT_NAME}`, which is the library name (in the `add_library` and other building steps).

After this, run `catkin_make` in the workspace folder. To run this node, first run `roscore` and then execute this node using

```bash
rosrun basic_cpp_libs simple_library_node
```

### BasicMathNode

| Field | Value |
| :--- | :--- |
| Name | `basic_math_node` |
| Code | [src/basic_math_node.cpp](./src/basic_math_node.cpp) |

A C++ node made to demonstrate using the `basic_math` [library](#basicmath) of this package.

#### Building and Running

To build and run this code, add the following lines to the `CMakeLists.txt` file

```makefile
add_executable(basic_math_node src/basic_math_node.cpp)
target_link_libraries(basic_math_node ${catkin_LIBRARIES} ${PROJECT_NAME}_basic_math)
```

Notice that the `target_link_libraries` for the `basic_math_node` executable links the `${PROJECT_NAME}_basic_math`, which is the library name for `basic_math` library in this package (described in the `add_library` for it).

After this, run `catkin_make` in the workspace folder. To run this node, first run `roscore` and then execute this node using

```bash
rosrun basic_cpp_libs basic_math_node
```

## Reference

- How to build and install C++ headers [through catkin](https://docs.ros.org/en/latest/api/catkin/html/howto/format2/building_libraries.html)
- List of variables [defined in catkin](https://docs.ros.org/en/latest/api/catkin/html/user_guide/variables.html)
