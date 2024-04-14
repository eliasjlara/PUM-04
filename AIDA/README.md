## Introduction
This part of the project is using **ROS2** with the version Humble Hawksbill.
**ROS2** is a middleware software for communication between different *Nodes* using topics and services.
To communicate using a topic we need at least one *publisher* and one *subscriber*. For example so can the publisher node be
a *C++* node and the subscriber a *Python* node. The only things needed for the communication to work is that the nodes need to communicate 
using the same topic (with the same namespace) and a mutual message type. Message are built using **C-style** variables.  
ROS2 is available for Windows, Mac and Linux. This project is mostly tested using ROS2 for Linux and we are currently only using *Python* nodes.

## Instructions
When starting a new terminal run the command:
`source /opt/ros/humble/setup.bash` or add to the command to *.bashrc* file  
### Creating new packages
**Python**
- To create a new *Python* package run the following code in the *src* folder:
  `ros2 pkg create --build-type ament_python <package_name>`
- To create a new *Python* node and package at the same time run the following code in the *src* folder:
  `ros2 pkg create --build-type ament_python --node-name <node_name> <package_name>`

**C++**
- To create a new *C++* package run the following code in the *src* folder: 
  `ros2 pkg create --build-type ament_cmake <package_name>`
- To create a new *C++* pnode and package at the same time run the following code in the *src* folder:
  `ros2 pkg create --build-type ament_cmake --node-name <node_name> <package_name>`  

Thereafter run the command `colcon build` or `colcon build --packages-select <package_name>` (to only build current package) in the *ros2_humble_ws* folder to setup the package.  

### Creating new nodes
**Python**  
- If package is already created add a new *Python* file in the folder src/package_name/package_name
- Go to *setup.py* file and add the `node_name = package_name.file_name:main` inside the console_scripts field as such:
  ```
  entry_points={
    `console_scripts`: [
      `node_name = package_name.file_name:main`,
      ]
  },
  ```
- If nodes are run on different workspaces, add the dependancies in the *package.xml* file using `<exec_depend>dependancy_name</exec_depend>`  

**C++**  
- If the package is already created add a new *C++* in the folder src/package_name/package_name
- In the *CMakeLists.txt* file add the following commands:
  ```
  find_package(dependancy_name REQUIRED)
  add_executable(node_name src/file_name.cpp)
  ament_target_dependencies(node_name dependancies)
  install(TARGETS
  node_name
  DESTINATION lib/${PROJECT_NAME})
  ```
- In the *package.xml* file add the dependancies using `<depend>dependancy_name</depend>`  

### Running nodes
To start a node in the project place yourself in the folder ros2_humble_ws (ros2 Humble Hawksbill workspace).  
- First build the changes using: `colcon build` or `colcon build --packages-select <package_name>` (for building current package only)  
- Thereafter source the local environment: `source install/localsetup.bash`  
- Last run the command: `ros2 run <package_name> <executable_name>`  

### Example
Example to start the Speech to text node in the file *faster_whisper_node.py* run the command:
```
colcon build
source install/localsetup.bash  
ros2 run speech_to_text faster_whisper_node
```

## References
For more information regarding ROS2 please visit [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html)
