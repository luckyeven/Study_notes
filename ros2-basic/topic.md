---
description: Topic is the bridge between nodes
---

# Topic

### Publisher/Subscriber

<figure><img src="https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.4_%E8%AF%9D%E9%A2%98/image8.gif" alt=""><figcaption></figcaption></figure>

Many-to-Many communication

<figure><img src="https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.4_%E8%AF%9D%E9%A2%98/image9.gif" alt=""><figcaption></figcaption></figure>

### Asynchronous Communication

Asynchronous means that after the publisher sends out the data, they do not know when the subscriber can receive it.

The asynchronous nature of topics makes them more suitable for publishing data with periodic updates, such as sensor data and motion control instructions. However, for certain instructions with strong logicality, such as modifying a parameter, using topics for transmission may not be very appropriate.

### Message Interface

Messages are a way of defining interfaces in ROS that are independent of programming languages. We can define them ourselves using files with the ".msg" suffix

### Example

{% code title="Publisher" %}
```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rclpy                                     # ROS2 Python interface lib
from rclpy.node import Node                      # ROS2 Node class
from std_msgs.msg import String                  # String class


class PublisherNode(Node):

    def __init__(self, name):
        super().__init__(name)                                    # ROS2 Node parent class init
        self.pub = self.create_publisher(String, "chatter", 10)   # Create a publisher object (message type, topic name, queue length).
        self.timer = self.create_timer(0.5, self.timer_callback)  # Create a timer (period in seconds, callback function to be executed at each timer tick).
    def timer_callback(self):                                     # Create the callback function to be executed periodically by the timer
        msg = String()                                            # Create a message object of type "String".
        msg.data = 'Hello World'                                  # Fill the message object with the desired message data
        self.pub.publish(msg)                                     # Publish the message on the topic.
        self.get_logger().info('Publishing: "%s"' % msg.data)     # Output a log message indicating that the topic has been published.

def main(args=None):                                 # ROS2 node main entry function.
    rclpy.init(args=args)                            # Initialize ROS2 Python API.
    node = PublisherNode("topic_helloworld_pub")     # Create and initialize the ROS2 node object.
    rclpy.spin(node)                                 # Loop and wait for ROS2 to exit.
    node.destroy_node()                              # Destroy the node object.
    rclpy.shutdown()                                 # Shut down the ROS2 Python API.

```
{% endcode %}

Open the `setup.py` file of the package and add the following entry point configuration:

```
    entry_points={
        'console_scripts': [
         'topic_helloworld_pub  = learning_topic.topic_helloworld_pub:main',
        ],
    },
```

{% code title="Subscriber" %}
```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy                                     # ROS2 Python interface lib
from rclpy.node import Node                      # ROS2 Node class
from std_msgs.msg import String                  # String class

class SubscriberNode(Node):

    def __init__(self, name):
        super().__init__(name)                             # ROS2 Node parent class init
        self.sub = self.create_subscription(\                
            String, "chatter", self.listener_callback, 10) # Create a subscriber object (message type, topic name,callback function ,queue length).

    def listener_callback(self, msg):                      # Create a callback function to process the data after receiving a topic message.
        self.get_logger().info('I heard: "%s"' % msg.data) # Output a log message to indicate the subscribed topic message has been received.

def main(args=None):                             
    rclpy.init(args=args)                          
    node = SubscriberNode("topic_helloworld_sub") 
    rclpy.spin(node)                              
    node.destroy_node()                            
    rclpy.shutdown()                               

```
{% endcode %}

Open the `setup.py` file of the package and add the following entry point configuration:

```
    entry_points={
        'console_scripts': [
         'topic_helloworld_pub  = learning_topic.topic_helloworld_pub:main',
         'topic_helloworld_sub  = learning_topic.topic_helloworld_sub:main',
        ],
    },
```

### Common commends

```
$ ros2 topic list                
$ ros2 topic info <topic_name> --verbose
$ ros2 topic hz <topic_name>    
$ ros2 topic bw <topic_name>    
$ ros2 topic echo <topic_name>   
$ ros2 topic pub <topic_name> <msg_type> <msg_data>   
```
