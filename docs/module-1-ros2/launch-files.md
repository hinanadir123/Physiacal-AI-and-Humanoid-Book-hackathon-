---
id: launch-files
title: Launch Files
---

Launch files are a way to start multiple ROS 2 nodes at once. They are written in Python and can be used to configure nodes, set parameters, and remap topics.

The following is an example of a launch file that starts two nodes: a publisher and a subscriber.

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_publisher',
            name='my_publisher'
        ),
        Node(
            package='my_package',
            executable='my_subscriber',
            name='my_subscriber'
        ),
    ])
```
