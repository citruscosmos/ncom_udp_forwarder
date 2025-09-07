# **NCOM UDP Forwarder for ROS2**

## **Overview**

This ROS2 package provides a simple yet effective node, ncom_udp_forwarder, designed to subscribe to an oxts_msgs/msg/Ncom topic and forward the raw NCOM data payload as a UDP packet. This is particularly useful for interfacing with real-time visualization tools like OxTS's NAVdisplay while replaying data from a rosbag.

The package includes a flexible launch file that allows for easy configuration of topic names, destination IP addresses, and ports.

## **Dependencies**

* ROS2 Humble Hawksbill (or compatible)  
* oxts_msgs package from the [oxts_ros2_driver](https://github.com/OxfordTechnicalSolutions/oxts_ros2_driver)

## **Build Instructions**

1. Clone this package into your ROS2 workspace's src directory.  
2. Ensure all dependencies are installed.  
3. Build the package using colcon:  
    ```shell
    cd /path/to/your/ros2_ws  
    colcon build --packages-select ncom_udp_forwarder
    ```

4. Source the workspace:  
    ```shell
    source install/setup.bash
    ```

## **Usage (ncom_udp_forwarder.launch.py)**

This package provides a launch file to start the ncom_forwarder_node.

To run with default settings:  
(Subscribes to /oxts/ncom, sends to 127.0.0.1:3000)  
  ```shell
  ros2 launch ncom_udp_forwarder ncom_udp_forwarder.launch.py
  ```

**Launch Arguments:**

* ncom_topic (string, default: /oxts/ncom): The topic publishing oxts_msgs/msg/Ncom.  
* udp_address (string, default: 127.0.0.1): The destination IP address for UDP packets.  
* udp_port (string, default: 3000): The destination UDP port.

**Example with custom arguments:**

  ```shell
  ros2 launch ncom_udp_forwarder ncom_udp_forwarder.launch.py ncom_topic:=/my_oxts/ncom udp_address:=192.168.1.100 udp_port:=3001
  ```

## **Interfacing with NAVdisplay**

To visualize the NCOM data from a rosbag in NAVdisplay, follow these steps:

1. Run the ROS2 Node:  
   Start the ncom_udp_forwarder node. If NAVdisplay is running on the same computer as your ROS2 workspace, the default settings are sufficient.  
    ```shell
   ros2 launch ncom_udp_forwarder ncom_udp_forwarder.launch.py
    ```

   This command forwards NCOM packets to udp://127.0.0.1:3000. If NAVdisplay is on a different machine, update the udp_address argument to that machine's IP address.  

2. Play the Rosbag:  
   In a separate terminal (with the ROS2 workspace sourced), play the rosbag file that contains the /oxts/ncom topic.  
    ```shell
   ros2 bag play /path/to/your/rosbag --clock 100
    ```
3. Open NAVdisplay:  
   a. Open NAVdisplay application.  
   b. You will see the device running the ncom_udp_forwarder node on Device association dialog and choose it.  

4. Verify Connection:  
   NAVdisplay should now start receiving data and display the vehicle's position, orientation, and other status information on its interface.

## **Node Details**

### **ncom_forwarder_node**

* **Subscribed Topics:**  
  * The topic name is set via the ncom_topic parameter (oxts_msgs/msg/Ncom).  
* **Published Topics:**  
  * None. The node outputs data via a UDP socket, not ROS topics.  
* **Parameters:**  
  * ncom_topic (string): See Launch Arguments.  
  * udp_address (string): See Launch Arguments.  
  * udp_port (int): See Launch Arguments.