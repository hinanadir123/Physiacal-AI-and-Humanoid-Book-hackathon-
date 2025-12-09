---
id: rclpy-python-nodes
title: rclpy (Python nodes)
---

`rclpy` is the ROS 2 client library for Python. It provides a Pythonic interface to the ROS 2 middleware. With `rclpy`, you can create nodes, publish and subscribe to topics, and use services.

The following is a simple example of a Python node that publishes a "Hello, ROS 2!" message to a topic.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher')
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS 2!'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    my_publisher = MyPublisher()
    rclpy.spin(my_publisher)
    my_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
