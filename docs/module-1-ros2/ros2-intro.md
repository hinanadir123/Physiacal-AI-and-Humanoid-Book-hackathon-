---
id: ros2-intro
title: ROS 2 Intro
sidebar_label: ROS 2 Intro
---

## Introduction to ROS 2

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of building complex and robust robot applications. ROS 2 is a complete rewrite of ROS 1, designed to address the limitations of its predecessor, especially in areas like real-time control, multi-robot systems, and embedded platforms.

### Key Features:
- **Distributed System:** ROS 2 supports distributed computing, allowing different parts of a robot's software to run on different machines.
- **Quality of Service (QoS):** Provides fine-grained control over communication parameters, essential for real-time applications.
- **Security:** Built-in security features for authentication, encryption, and access control.
- **Client Libraries:** Supports multiple programming languages, including Python (rclpy) and C++ (rclcpp).

## Core ROS 2 Concepts

### Nodes

A **Node** is a process that performs computation. In ROS 2, a node is the fundamental unit of execution. Typically, a ROS system consists of many nodes, each responsible for a specific task (e.g., controlling a motor, reading sensor data, performing navigation). Nodes communicate with each other using various mechanisms provided by ROS 2.

### Topics

**Topics** are a fundamental communication mechanism in ROS 2. They implement an anonymous publish/subscribe model. This means that a node can publish messages to a named topic without knowing which other nodes (subscribers) are receiving those messages. Similarly, a node can subscribe to a topic to receive messages without knowing which node is publishing them.

-   **Publisher:** A node that sends messages to a topic.
-   ****Subscriber:** A node that receives messages from a topic.

### Services

**Services** provide a request/reply communication pattern. Unlike topics, which are a one-way broadcast, services allow a client node to send a request to a service server node and wait for a response. This is useful for functionalities that require a direct answer or an immediate action, such as commanding a robot to perform a specific action and receiving confirmation.

## Example Python Snippet (rclpy Node)

Here's a simple Python example using `rclpy` to create a basic ROS 2 node that periodically prints "Hello from ROS 2!" to the console. This demonstrates the creation of a node and the use of a timer to trigger actions.

```python
import rclpy
from rclpy.node import Node

class MySimpleNode(Node):
    def __init__(self):
        super().__init__('my_simple_node') # Create a node with the name 'my_simple_node'
        self.timer = self.create_timer(1.0, self.timer_callback) # Create a timer that calls timer_callback every 1 second
        self.get_logger().info('MySimpleNode has been started!')

    def timer_callback(self):
        self.get_logger().info('Hello from ROS 2!') # Log a message

def main(args=None):
    rclpy.init(args=args)         # Initialize rclpy
    node = MySimpleNode()         # Create the node
    rclpy.spin(node)              # Keep the node alive until it's explicitly stopped or Ctrl+C is pressed
    node.destroy_node()           # Destroy the node explicitly
    rclpy.shutdown()              # Shutdown rclpy

if __name__ == '__main__':
    main()
```
This example shows the basic structure of a Python ROS 2 node:
1.  **`rclpy.init()` and `rclpy.shutdown()`**: Initialize and deinitialize the ROS 2 client library.
2.  **`Node` class inheritance**: Your custom node class inherits from `rclpy.node.Node`.
3.  **`super().__init__('node_name')`**: Calls the base class constructor and assigns a name to your node.
4.  **`self.create_timer()`**: Sets up a timer to execute a callback function at regular intervals.
5.  **`self.get_logger().info()`**: Used for logging messages from the node.
6.  **`rclpy.spin()`**: Keeps the node running, processing callbacks until it's shut down.