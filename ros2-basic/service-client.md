# Service/Client

<figure><img src="https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.5_%E6%9C%8D%E5%8A%A1/image8.gif" alt=""><figcaption></figcaption></figure>

* Synchronous Communication
* One-to-Many

<figure><img src="https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.5_%E6%9C%8D%E5%8A%A1/image9.gif" alt=""><figcaption></figcaption></figure>

### Service Interface

Similar to topic-based communication, the core of service-based communication is still to transfer data, which is divided into two parts: the request data and the feedback data. These data, like topic messages, must also be defined according to standard in ROS. Topics use ".msg" files to define message types, while services use ".srv" files to define service types.

```
$ ros2 service list              
$ ros2 service type <service_name>  
$ ros2 service call <service_name> <service_type> <service_data>  

```

