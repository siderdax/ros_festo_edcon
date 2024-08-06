# ROS2 festo motion controller

control CMMT serial using fest-edcon



## Build

```bash
colcon build --symlink-install
```



## Run

```bash
source install/setup.bash
ros2 run ros_festo_edcon edcon --ros-args -p edcon_config.host:=<ip address>
```
