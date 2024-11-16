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
3. **Running the service client node**

    To run the subscriber node, run the following command 
    ```sh
    ros2 run beginner_tutorials server_client "Changed Output" --ros-args --log-level debug
    ```

4. **Running the launch file**

    ```sh
    ros2 launch beginner_tutorials launch.yaml frequency:=1
    ```
    

## TF Frame 
The `talker` node in beginner_tutorials package can now broadcasts TF frame. The `publisher_member_function.cpp` file implements broadcasting in particular a static transform using the from world to talker frame. This transform can be visualized in tools like RViz.

To broadcast a TF frame, the `talker` node publishes a transform with the following details:

- **Parent frame**: "world"
- **Child frame**: "talker"
- **Transform**: A static transform that places the `talker` at a fixed position relative to the `world` frame.

This function is called periodically within the `talker` node to continuously broadcast the transform.

To run the publisher, use the following command:

```bash
ros2 run beginner_tutorials talker
```

**To view the TF transform, run the following commands in separate terminals:**
```bash
ros2 topic echo /tf_static

ros2 run tf2_tools view_frames
```

## Colcon Test File

The `test/test_node.cpp` file contains level 2 integration tests for the nodes in this package. These tests are designed to verify the functionality of the `talker` and `server_client` nodes, ensuring that they perform as expected.

### Running the Tests

To run the tests, follow these steps:

1. **Build the package with tests**:

    ```bash
    colcon build
    ```

2. **Run the tests**:

    ```bash
    colcon test
    ```

3. **View the test results**:
    ```bash
    cat log/latest_test/beginner_tutorials/stdout_stderr.log
    ```

The tests will output the results, indicating whether each test passed or failed.


## ROS2 Bag to Record and Replay

To record and replay topics, follow these steps:

1. **Build the workspace**:
    ```bash
    colcon build
    ```

2. **Source the setup**:
    ```bash
    source install/setup.bash
    ```

3. **Run the launch file to start recording**:
    ```bash
    ros2 launch beginner_tutorials bag.launch.py enable_recording:=True
    ```

    The recording will start and stop automatically after 15 seconds. Once the recording has stopped, the files `metadata.yaml` and `rosbag_0.db3` will be created under `ros2_ws/results/rosbag`.

4. **Play the recording**:
    ```bash
    ros2 bag play results/rosbag
    ```

5. **Run the listener node to see the recorded data**:
    ```bash
    ros2 run beginner_tutorials listener
    ```

6. **Echo the topic to view the messages**:
    ```bash
    ros2 topic echo /topic
    ```

## About the Nodes 

**Talker**

The ```talker``` node publishes messages to the ```topic``` topic


**Listener**

The ```listener``` node subscribes to messages from the ```topic``` topic.

**Server Client Node**

The ```server_client``` node functions as a client that sends requests to modify the message published by the talker node. it updates the base string that the talker node broadcasts to the topic.

## RQT Console Log Output

![Screenshot from 2024-11-09 00-04-12](https://github.com/user-attachments/assets/920c47f3-83a5-4954-8169-002cdf102ce5)

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
