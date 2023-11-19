# ros2_template

The ROS 2 repository template. This repository may be used entirely as a base for your repository. Alternatively, specific components may be incorporated into your already defined repository structure to help organize things or allow implement them faster.

## ros2_package

The ROS 2 C++ package template. A complete, able-to-build package with ROS-specific structures defined, following best practices of C++ and ROS 2 programming.

### Incorporates

- ROS 2 node template with API objects defined (subscriber, publisher, service server, service client)
- Node configuration file
- Node launch file
- The `CMakeFile.txt` divided into understandable sections
- The `package.xml` able to resolve all dependencies with a rosdep tool
- Readme template
- Unit tests structure template -> TODO: to be completed

### How to use

After building the package, the following custom API is available:

![Node API](./images/Screenshot%20from%202023-11-19%2018-03-15.png)

To easily configure the template to your needs, follow the steps:

- Rename files or directories:
  - [config file](./ros2_package/config/ros2_package.yaml)
  - [launch file](./ros2_package/launch/ros2_package.launch.py)
  - [include directory](./ros2_package/include/ros2_package/)
  - [node header](./ros2_package/include/ros2_package/ros2_package_node.hpp)
  - [node source code](./ros2_package/src/ros2_package_node.cpp)
  - [node testing code](./ros2_package/test/ros2_package_node_test.cpp)
- Find and replace:
  - Changed files or directory names
  - All occurrences of the package and node names
  - A `namespace` with your project's namespace
- Modify source the code and other package components accordingly to your needs

_**Note:** It is possible to easily replace all occurrences of a given sentence in vscode with `ctrl+shit+h` shortcut._
