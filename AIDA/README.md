# AIDA - code for robot
## Introduction
This part of the project is supposed to be placed on the Raspberry Pis on the robot AIDA. This part is using the **Humble Hawksbill** version of **ROS2**. ROS2 is a middleware software for communication between different *Nodes* using *topics* and *services*. To communicate using a topic we need at least one *publisher* and one *subscriber*. One of the main benefits of using ROS2 for communication between is that we only need to focus on the nodes publishing and subscribing to the same topic and that they are using a mutual message type. ROS2 takes care of the rest, as nodes that communicate to the same topic can even be written in different lanuages. ROS2 has main support for nodes written in **Python** and **C++**. In this project we are only using **Python** nodes. Message are built using **C-type** variables. Besides publishers and subscribers there are also other types of communication available for ROS2. In this project we are also using services for shutting down and turning on steams from hardware such as the microphone and the camera.  
ROS2 is available for Windows, Mac and Linux. This project aims to support at least the Linux version of ROS2 as this is the operating system of AIDA and what most developers in the project are using for developing and testing the code. 

## Layout under src folder
### Speech to text  
Nodes and logic for the speech to text part of AIDA.  
### Audio  
Nodes and logic for recording of audio data by microphone.   
### Audio package  
Messages for topic communication of audiodata.  
### Aida interfaces  
Setup for using services with ROS2.  

## Instructions for ROS2
When starting a new terminal run the command:
`source /opt/ros/humble/setup.bash` or add the command to *.bashrc* file  
### Creating new packages
**Python**
- To create a new **Python** package run the following code in the *src* folder:
  `ros2 pkg create --build-type ament_python <package_name>`
- To create a new **Python** node and package at the same time run the following code in the *src* folder:
  `ros2 pkg create --build-type ament_python --node-name <node_name> <package_name>`

**C++**
- To create a new **C++** package run the following code in the *src* folder: 
  `ros2 pkg create --build-type ament_cmake <package_name>`
- To create a new **C++** node and package at the same time run the following code in the *src* folder:
  `ros2 pkg create --build-type ament_cmake --node-name <node_name> <package_name>`  

Thereafter run the command `colcon build` or `colcon build --packages-select <package_name>` (to only build current package) in the *ros2_humble_ws* folder to setup the package.  

### Creating new nodes
**Python**  
- If package is already created add a new **Python** file in the folder *src/package_name/package_name*
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
- If the package is already created add a new **C++** in the folder *src/package_name/package_name*
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
### Tips
>[!NOTE]
>The topic of publisher and subscriber must be in the same namespace

>[!NOTE]
>It is easier to install ROS2 for Linux compared to Windows and Mac. Communication between different nodes in the same workspace works on VirtualBox with Ubuntu so that is an alternative for testing on computer instead of AIDA. 

## References
For more information regarding ROS2 please visit [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html)
