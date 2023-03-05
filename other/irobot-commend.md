# irobot commend

{% code title="service" %}
```
# linear movement
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -1,y: 0, z: 0}}"

```
{% endcode %}

{% code title="action" %}
```
# docking
ros2 action send_goal /dock irobot_create_msgs/action/Dock "{}"
# undocking
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"

# Drive Distance
ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance "{distance: 0.5,max_translation_speed: 0.15}"

# Drive Arc
ros2 action send_goal /drive_arc irobot_create_msgs/action/DriveArc "{angle: 1.57,radius: 0.3,translate_direction: 1,max_translation_speed: 0.3}"

# Rotate
ros2 action send_goal /rotate_angle irobot_create_msgs/action/RotateAngle "{angle: 1.57,max_rotation_speed: 0.5}"
```
{% endcode %}

{% code title="action" %}
```
```
{% endcode %}
