---
description: >-
  Actions are one of the communication types in ROS 2 and are intended for long
  running tasks. They consist of three parts: a goal, feedback, and a result.
---

# Action

<figure><img src="https://docs.ros.org/en/humble/_images/Action-SingleActionClient.gif" alt=""><figcaption></figcaption></figure>

* Actions use client/server model
* Synchronous communication
* One Server, multi-clients
* .action file defines the interface

```bash
ros2 run action list

ros2 action info </action>

ros2 action send_goal </action_name> </action_type> <"{theta: 3.14}"> --feedback
```
