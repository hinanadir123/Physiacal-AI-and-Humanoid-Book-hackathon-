---
id: urdf-links-joints
title: URDF (links, joints)
---

URDF (Unified Robot Description Format) is an XML format for representing a robot model. URDF can be used to describe the physical properties of a robot, such as its links, joints, and sensors.

## Links

Links are the rigid parts of a robot. They are connected by joints.

```xml
<link name="my_link">
  <visual>
    <geometry>
      <box size="0.1 0.2 0.3" />
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.1 0.2 0.3" />
    </geometry>
  </collision>
  <inertial>
    <mass value="1.0" />
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0" />
  </inertial>
</link>
```

## Joints

Joints connect links together. They can be revolute, prismatic, or fixed.

```xml
<joint name="my_joint" type="revolute">
  <parent link="parent_link" />
  <child link="child_link" />
  <axis xyz="0 0 1" />
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1" />
</joint>
```
