# my_beginner_tutorials

## Overview
This repository contains beginner tutorials for ROS2 Humble. It includes minimal modified publisher and subscriber nodes to demonstrate pub sub communication in ROS2

## Author 
Amogha Thalihalla Sunil (amoghats@umd.edu)

## Prerequisites
- ROS2 Humble installed
- Colcon build tool
- C++17 compatible compiler

## Installation and building the setup

1. **Create a ROS Workspace if not already created**
    ```sh 
    cd 
    mkdir -p ros2_ws/src/
    cd ~/ros2_ws/src/ 
    ```

2. **Create a ROS package "Beginner_tutorials**
    ```sh 
    ros2 pkg create --build-type ament_cmake --license Apache-2.0 beginner_tutorials
    ```

3. **Download the Source Code for both publisher and subscriber**
    ```sh 
    cd beginner_tutorials/src/
    wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_publisher/member_function.cpp
    wget -O subsriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_subscriber/member_function.cpp
    ```

4. **Check for Missing Dependencies Before Building**

    To run rosdep in the root of the workspace: 

    ```sh
    cd ~/ros2_ws 
    rosdep install -i --from-path src --rosdistro humble -y
    ```

5. **Build the package**
    Use colcon to build the package 

    ```sh
    colcon build --packages-select beginner_tutorials
    ```

4. **Source the setup**

    Source the script setup to overlay this workspace on the environment 
    ```sh
    source install/setup.bash
    ```

## Using the package / Running the Package 

1. **Running the publisher node** 

    To run the talker node, run the following command 

    ```sh
    ros2 run beginner_tutorials talker
    ```

2. **Running the subscriber node**

    To run the subscriber node, run the following command 
    ```sh
    #Open a new terminal and source the setup in this terminal
    ros2 run beginner_tutorials listener
    ```
2. **Running the service client node**

    To run the subscriber node, run the following command 
    ```sh
    ros2 run beginner_tutorials server_client "Changed Output" --ros-args --log-level debug
    ```


## About the Nodes 

**Talker**

The ```talker``` node publishes messages to the ```topic``` topic


**Listener**

The ```listener``` node subscribes to messages from the ```topic``` topic.

**Server Client Node**
The server_client node functions as a client that sends requests to modify the message published by the talker node. it updates the base string that the talker node broadcasts to the topic.

## RQT Console Log Output


## Linting

cpplint has been run and the output is saved in the ```cpplintOutput.txt``` file

To run cpplint run the following command :

```sh
cpplint --filter="-legal/copyright" $( find . -name *.cpp | grep -v "/build/" )
```

## Clangd formatting
clangd formatting is checked and has no more changes required.
Screenshot is attached.

```sh
clang-format -style=Google -i src/beginner_tutorials/src/*.cpp
```

## License

This project is licensed under the BSD-3-Clause Licence. Check the license file for details

## Acknowledgements 

- Open Source Robotics Foundation, Inc.
- ROS2 Community