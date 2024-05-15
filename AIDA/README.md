# AIDA - code for robot
## Introduction
This part of the project is supposed to be placed on the Raspberry Pis on the robot AIDA. This part is using the **Humble Hawksbill** version of **ROS2**. ROS2 is a middleware software for communication between different *Nodes* using *topics* and *services*. To communicate using a topic we need at least one *publisher* and one *subscriber*. One of the main benefits of using ROS2 for communication between is that we only need to focus on the nodes publishing and subscribing to the same topic and that they are using a mutual message type. ROS2 takes care of the rest, as nodes that communicate to the same topic can even be written in different lanuages. ROS2 has main support for nodes written in **Python** and **C++**. In this project we are only using **Python** nodes. Message are built using **C-type** variables. Besides publishers and subscribers there are also other types of communication available for ROS2. In this project we are also using services for shutting down and turning on steams from hardware such as the microphone and the camera.  
ROS2 is available for Windows, Mac and Linux. This project aims to support at least the Linux version of ROS2 as this is the operating system of AIDA and what most developers in the project are using for developing and testing the code. 

## Dependancies needed for the project
In the project we are using several dependacies which are available using **Pythons** package manager **pip**. Run these before building the project. 
```
pip install faster-whisper
pip install numpy
pip install sounddevice
pip install mediapipe
pip install pytest
```

## Layout under src folder
**speech_to_text**  
Nodes and logic for the speech to text part of AIDA.  

**audio**  
Nodes and logic for recording of audio data by microphone.   

**audio_package**  
Messages for topic communication of audiodata.  

**aida_interfaces**  
Setup for using services with ROS2.  

## Main principles for ROS2
There are 3 main ways to communicate by *topics* in ROS2. 
- Publisher and subscriber
- Client and service
<!-- - Action server and client   -->
### Publisher and subscriber
For most of the nodes in the project we are using publishers and subscribers. The main functionality of the publisher and subsriber are: 
- The publishers main objective is to publish information to the topic.  
- The subscriber lies and waits until new information is found in the topic. When new information is available, the callback function is triggered which has an argument called **msg**. This message contains the information published to the topic.  
For more information about a basic publisher and subscriber, see [Publisher and subscriber in Python](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html).    
### Client and service  
The other way to communicate between nodes is by using services. The main functionality of the service and client are: 
- The client is the "main" node which calls for new information in the client server design pattern. This is done by sending a request to the topic. When the service publishes a response the client receives it and continues with its operation. 
- The service lies and waits for a new request from the client. When a request is published to the topic the service starts working and publishes its result in something called a response.  
For more information about a simple client server, see [Simple client server in Python](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)  
<!-- (### Action server and client , we may need to add information about this one, but are we actually using this in our project?) -->
<!-- This is how you make a comment in markdown-->




## Instructions for ROS2
When starting a new terminal run the command `source /opt/ros/humble/setup.bash` or add the command to the *.bashrc* file  
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

### Launching a group of nodes
To launch a group of nodes place yourself in the folder ros2_humble_ws (ros2 Humble Hawksbill workspace).  
- First build the changes using: `colcon build` or `colcon build --packages-select <package_name>` (for building current package only)  
- Thereafter source the local environment: `source install/localsetup.bash`
- Move to the ros2_humble_ws/launch directory.
- Run the command: `ros2 launch <launch_file_name.yaml>`

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


To test a service request in command line use the following syntax `ros2 service call /mic/SetState aida_interfaces/srv/SetState "{desired_state: 'active'}"`

### Docker
To run nodes in docker (on linux) simply move to the *PUM-04/AIDA/docker/* directory and build the container using:
```
sudo docker build -t ros2-aida --build-arg CACHEBUST=$(date +%s) .
```
and to actually run them:
```
sudo docker run -it --rm --device=/dev/video0 --device=/dev/snd/controlC0 --device=/dev/snd/pcmC0D0c --network host ros2-aida
```
The above command runs the container with an interactive environment, allows access to camera and microphone, and shares the network with host computer. If the access to either camera or microphone doesn't work, try to find the name of your device in */dev/* or */dev/snd/* and replace suiting part above command..

### Tips
>[!NOTE]
>The topic of publisher and subscriber must be in the same namespace

>[!NOTE]
>It is easier to install ROS2 for Linux compared to Windows and Mac. Communication between different nodes in the same workspace works on VirtualBox with Ubuntu so that is an alternative for testing on computer instead of AIDA. 

## References
For more information regarding ROS2 please visit [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html)
