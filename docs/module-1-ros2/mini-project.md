---
id: mini-project
title: Mini Project (simple node + URDF)
---

In this mini project, you will create a simple ROS 2 package that contains a node that publishes the state of a URDF model.

## 1. Create a package

```bash
ros2 pkg create my_robot_pkg --build-type ament_python
```

## 2. Create a URDF file

Create a file named `my_robot.urdf` in the `my_robot_pkg` directory with the following content:

```xml
<robot name="my_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2" />
      </geometry>
    </visual>
  </link>
  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05" />
      </geometry>
    </visual>
  </link>
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link" />
    <child link="wheel_link" />
    <axis xyz="0 1 0" />
  </joint>
</robot>
```

## 3. Create a state publisher node

Create a file named `state_publisher.py` in the `my_robot_pkg` directory with the following content:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class StatePublisher(Node):
    def __init__(self):
        super().__init__('state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.joint_position = 0.0

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['base_to_wheel']
        msg.position = [self.joint_position]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing joint states')
        self.joint_position += 0.1

def main(args=None):
    rclpy.init(args=args)
    state_publisher = StatePublisher()
    rclpy.spin(state_publisher)
    state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 4. Build and run the package

```bash
colcon build --packages-select my_robot_pkg
source install/setup.bash
ros2 run my_robot_pkg state_publisher
```

## 5. Visualize the robot

In a new terminal, run the following command to visualize the robot in RViz2:

```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix my_robot_pkg)/share/my_robot_pkg/my_robot.urdf
```
