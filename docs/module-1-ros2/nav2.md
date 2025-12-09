---
id: nav2
title: Nav2
sidebar_label: Nav2
---

import {useEffect} from 'react';

<div className="animated-section">

## 🚶‍♂️ Chapter 3: Nav2 for Bipedal Humanoids

Nav2 is the second generation of the ROS Navigation Stack. While it's commonly used for wheeled robots, it can be adapted for more complex robots like bipedal humanoids. It provides a flexible and powerful framework for autonomous navigation.

### Key Concepts for Bipedal Navigation 🤖

-   **Path Planning:** Nav2's planners (like SmacPlanner) can generate a 2D path from the robot's current location to a goal. For a bipedal robot, this path is then used as a reference for the robot's walking pattern generator.
-   **Footstep Planning:** A crucial step for bipedal robots is to plan the placement of each foot. This is often handled by a specialized planner that takes the 2D path from Nav2 and generates a sequence of footstep goals.
-   **Collision Avoidance:** Nav2's costmaps are used to represent the environment and avoid obstacles. For a bipedal robot, the costmap can be used to ensure that the planned footsteps are in safe, non-colliding locations.
-   **Whole-Body Control:** The final step is to use a whole-body controller to execute the planned footsteps, ensuring the robot maintains its balance while walking.

### Example: Simplified Path Planning

While a full bipedal navigation system is complex, here is a simplified example of how you might send a goal to Nav2 using the ROS 2 CLI.

```bash
# Send a navigation goal to Nav2
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  'pose': {
    'header': {
      'frame_id': 'map'
    },
    'pose': {
      'position': {
        'x': 1.0,
        'y': 0.5,
        'z': 0.0
      },
      'orientation': {
        'w': 1.0
      }
    }
  }
}"
```

This command tells Nav2 to plan a path to the goal coordinates (1.0, 0.5) in the `map` frame. For a bipedal robot, a separate system would then take this path and convert it into a sequence of walking motions. 🚶‍♀️

</div>

<style>
{`
.animated-section {
  opacity: 0;
  transform: translateY(20px);
  animation: slide-in 0.5s ease-out forwards;
}

@keyframes slide-in {
  to {
    opacity: 1;
    transform: translateY(0);
  }
}
`}
</style>
