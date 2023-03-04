# Package

### Create package

```
ros2 pkg create --build-type <build-type> <package_name>

# example
cd ~/dev_ws/src
ros2 pkg create --build-type ament_python learning_pkg_python # Python
```

### Compile package

```
cd ~/dev_ws
colcon build   
source install/local_setup.bash
```

