# Report

## Introduction

In this assignment, I will present the implementation of avoiding obstacles using an iRobot, specifically, I used tape to create a boundary area, which restricts the robot's movement within a specific area.&#x20;

## Equipment

* Laptop(Ubuntu 22.04)&#x20;
* PC (Ubuntu 22.04)
* Irobot create3
*   Other libraries and software:

    * ROS2 Humble
    * rqt
    * ssh

    ##

    ## High-Level Design Graph

    <figure><img src="../.gitbook/assets/Blank diagram (1).png" alt=""><figcaption></figcaption></figure>

The Irobot has a relatively small volume and is more suitable for placing a Raspberry Pi on it. The Raspberry Pi is the controller to mainly implements functions such as sensor driving and motor control. However, visual processing and application functions are not suitable for running on the Raspberry Pi due to the low memory, so I put them on another more powerful laptop.

The communication between the two computers seems a bit complicated, after all, there are quite a lot of data transmitted between them. However, the ROS system has already been designed for us, and we only need to configure the ROS environment on each computer. There is no need to make any changes in function development, and the implementation is very convenient.

In addition to the PC I used, I chose a laptop as another computing platform to simulate a Raspberry Pi as a controller placed on the robot.

## Low-Level Design Graph

## Procedure

#### Create workspace

```
$ mkdir -p dev_ws_src/src
$ cd ~/dev_ws/src

# create vision package
$ ros2 pkg create --build-type ament_python vision_topic
```

#### webcam\_pub.py

````
```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy                        # ROS2 Python interface lib
from rclpy.node import Node         # ROS2 Node class
from sensor_msgs.msg import Image   # image message type
from cv_bridge import CvBridge      # ROS image message to OpenCV image converter
import cv2                          # Opencv lib

"""
ROS2 ImagePublisher class
"""
class ImagePublisher(Node):

    def __init__(self, name):
        super().__init__(name)                                           # ROS Node init
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)  # create image topic publisher
        self.timer = self.create_timer(0.1, self.timer_callback)         # create timer to publish image topic
        self.cap = cv2.VideoCapture(0)                                   # create video capture object
        self.cv_bridge = CvBridge()                                      # create cv_bridge object

    def timer_callback(self):
        ret, frame = self.cap.read()                                     # read video frame
        
        if ret == True:                                                  # if read video frame successfully
            self.publisher_.publish(
                self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8'))             # publish video frame to topic

        self.get_logger().info('Publishing video frame')                 # print log info

def main(args=None):                                 # main function
    rclpy.init(args=args)                            # ROS2 Python interface init
    node = ImagePublisher("topic_webcam_pub")        # create ROS2 Node object
    rclpy.spin(node)                                 # spin node
    node.destroy_node()                              # destroy node
    rclpy.shutdown()                                 # ROS2 Python interface shutdown

```
````

After completing the code implementation, It is needed to set the compilation options for the package to let the system know the entry point of the Python program. To do this, open the `setup.py` file of the package and add the following configuration for the entry points:

```yaml
entry_points={
        'console_scripts': [
         'webcam_pub  = vision_topic.webcam_pub:main',
        ],
    },
```

Run the `webcam_pub` topic and `rqt` tool to see video frames.

<figure><img src="../.gitbook/assets/image.png" alt=""><figcaption><p>Video captured</p></figcaption></figure>

#### webcam\_sub.py

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rclpy                            # ROS2 Python library
from rclpy.node import Node             # ROS2 node class
from sensor_msgs.msg import Image       # ROS2 image message
from cv_bridge import CvBridge          # ROS2 OpenCV bridge
import cv2                              # Opencv library
import numpy as np                      # Python library for numerical computation

lower_red = np.array([0, 90, 128])      # the lower boundary of red color in HSV color space
upper_red = np.array([180, 255, 255])   # the upper boundary of red color in HSV color space


"""
create a class to define a ROS2 node
"""
class ImageSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)                                  # ROS2 parent class initialization
        self.sub = self.create_subscription(
            Image, 'image_raw', self.listener_callback, 10)     # create a subscriber to receive image message
        self.cv_bridge = CvBridge()                             # create a bridge to convert ROS2 image message to OpenCV image

        self.declare_parameter('red_h_upper', 0)                # declear a parameter
        self.declare_parameter('red_h_lower', 0)                # declear a parameter
    

    def edge_detect(self, image):
        upper_red[0] = self.get_parameter('red_h_upper').get_parameter_value().integer_value      # read the upper threshold parameter value
        lower_red[0] = self.get_parameter('red_h_lower').get_parameter_value().integer_value      # read the lower threshold parameter value
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)        # convert BGR image to HSV image
        mask_red = cv2.inRange(hsv_img, lower_red, upper_red)   # extract the red color region in the HSV image
        contours, hierarchy = cv2.findContours(
            mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)     # find the contours of the red color region

        for cnt in contours:                                    # traverse all the contours
            if cnt.shape[0] < 150:
                continue

            (x, y, w, h) = cv2.boundingRect(cnt)                # get the bounding rectangle of the contour
            cv2.drawContours(image, [cnt], -1, (0, 255, 0), 2)  # draw the contour on the image
            cv2.circle(image, (int(x+w/2), int(y+h/2)), 5,
                       (0, 255, 0), -1)                         # draw the center of the contour on the image

        cv2.imshow("object", image)                             # show the image
        cv2.waitKey(10)

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')         # print the log information
        image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')      # convert ROS2 image message to OpenCV image
        self.edge_detect(image)                               # call the edge detection function


def main(args=None):                                        # main function
    rclpy.init(args=args)                                   # initialize ROS2 Python interface
    node = ImageSubscriber("webcam_sub")              # create a node object
    rclpy.spin(node)                                        # spin the node
    node.destroy_node()                                     # destroy the node
    rclpy.shutdown()                                        # shutdown ROS2 Python interface

```

Add the entry point

```yaml
entry_points={
        'console_scripts': [
         'webcam_pub = vision_topic.webcam_pub:main',
         'webcam_sub = vision_topic.webcam_sub:main',
        ],
    },
```

By declaring two parameters in the codes, the threshold can be easily adjusted during run time without rebuilding the package using the following commends;

```bash
ros2 set webcam_sub red_h_lower 0
ros2 set webcam_sub red_h_upper 180
```

<figure><img src="../.gitbook/assets/image (1).png" alt=""><figcaption><p>Example of edge detection</p></figcaption></figure>
