---
id: nodes-topics-services
title: Nodes, Topics, Pub/Sub, Services
---

## Nodes

A node is a process that performs computation. In ROS 2, a node is a fundamental unit of execution. It can be a sensor driver, a motor controller, or a path planner.

## Topics

Topics are named buses over which nodes exchange messages. Topics have anonymous publish/subscribe semantics, which means that the producer of the data (the publisher) does not know who is consuming it (the subscriber).

### Publishing

A publisher is a node that writes data to a topic.

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

### Subscribing

A subscriber is a node that reads data from a topic.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MySubscriber(Node):
    def __init__(self):
        super().__init__('my_subscriber')
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    my_subscriber = MySubscriber()
    rclpy.spin(my_subscriber)
    my_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services

Services are a request/reply communication pattern. One node (the client) sends a request to another node (the server) and waits for a reply.

```python
# Service server
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class MyService(Node):
    def __init__(self):
        super().__init__('my_service')
        self.srv = self.create_service(SetBool, 'my_service', self.service_callback)

    def service_callback(self, request, response):
        self.get_logger().info('Incoming request: %s' % request.data)
        response.success = True
        response.message = "Service call successful"
        return response

def main(args=None):
    rclpy.init(args=args)
    my_service = MyService()
    rclpy.spin(my_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python
# Service client
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class MyClient(Node):
    def __init__(self):
        super().__init__('my_client')
        self.cli = self.create_client(SetBool, 'my_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self):
        self.req.data = True
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    my_client = MyClient()
    my_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(my_client)
        if my_client.future.done():
            try:
                response = my_client.future.result()
            except Exception as e:
                my_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                my_client.get_logger().info(
                    'Result of service call: %s' % (response.success,))
            break

    my_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
