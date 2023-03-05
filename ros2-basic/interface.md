---
description: >-
  ROS applications typically communicate through interfaces of one of three
  types: messages, services and actions
---

# Interface

### Topic

{% code title=".msg" %}
```
# data
int32 x
int32 y
```
{% endcode %}

### Service

{% code title=".srv" %}
```
# require data
int 64 a
int 64 b
---
response data
int64 sum
```
{% endcode %}

### Action

{% code title=".action" %}
```
# goal
bool enable
---
# result
bool finish
---
# feedback
int32 state
```
{% endcode %}

```
$ ros2 interface list                    
$ ros2 interface show <interface_name>   
$ ros2 interface package <package_name>  

```

{% embed url="https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html" %}

