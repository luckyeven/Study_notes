# Node

### Example

Creating a Hello World Node

{% code lineNumbers="true" %}
```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy                                     # ROS2 Python library
from rclpy.node import Node                  
import time

class HelloWorldNode(Node):
    def __init__(self, name):
        super().__init__(name)                     # ROS2 Node parent init
        while rclpy.ok():                          # ROS2 system is okay
            self.get_logger().info("Hello World")  # ROS2 log
            time.sleep(0.5)                        # wait time

def main(args=None):                               # ROS2 node main
    rclpy.init(args=args)                          # ROS2 Python interface init
    node = HelloWorldNode("node_helloworld_class") # ROS2 node objet init
    rclpy.spin(node)                               # wait for ROS2 quit
    node.destroy_node()                            # destroy node
    rclpy.shutdown()                               # close ROS2 Python interface

```
{% endcode %}

Open `setup.py` in the package, add configure;

```yaml
    entry_points={
        'console_scripts': [
         'node_helloworld       = learning_node.node_helloworld:main',
         'node_helloworld_class = learning_node.node_helloworld_class:main',
        ],

```
