# Packages for Learning ROS

All the packages in this repository.

## Table of contents

- [Packages for Learning ROS](#packages-for-learning-ros)
    - [Table of contents](#table-of-contents)
    - [Foreword](#foreword)
    - [Basic Packages](#basic-packages)
    - [References](#references)

## Foreword

This folder contains all the packages in this repository made for the purpose of learning ROS. Each package has a README of its own and may have a tutorial as well (depending upon what the contents are).

For most of the packages, specially those which have traditional coding involved, a C++ and a Python version are made as separate packages.

## Basic Packages

Essential packages that every beginner must know before proceeding with any ROS application

<table>
    <tr>
        <th> S.No. </th>
        <th> Title </th>
        <th colspan="2" style="text-align:center"> Package </th>
        <th> Description </th>
    </tr>
    <tr>
        <td> 1 </td>
        <td style="text-align:center"> Basic Nodes </td>
        <td> <a href="./cpp_basic_nodes/README.md">cpp_basic_nodes</a> </td>
        <td> <a href="./py_basic_nodes/README.md">py_basic_nodes</a> </td>
        <td>
            <b> Basic Nodes</b>: Hello world, Publisher, Subscriber, Service (Server and Client), Actions (Server and Client), Parameters, Dynamic Reconfigure
        </td>
    </tr>
    <tr>
        <td> 2 </td>
        <td style="text-align:center"> External Libraries </td>
        <td> <a href="./basic_cpp_libs/README.md"> basic_cpp_libs </a> </td>
        <td> <a href="./basic_py_libs/README.md"> basic_py_libs </a> </td>
        <td>
            <b> Basic Libraries and Modules </b>: A simple library (to compartmentalize complex codes and keep nodes simple)
        </td>
    </tr>
</table>

## References

- roscpp API documentation [here](https://docs.ros.org/en/api/roscpp/html/)
- rospy API documentation [here](http://docs.ros.org/en/melodic/api/rospy/html/)
