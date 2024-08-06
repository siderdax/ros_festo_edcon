# ROS2 festo motion controller

control CMMT serial using fest-edcon



## Build

1. Install ROS2
2. Clone git
3. Install fest-edcon
    ```bash
    sudo apt install python3-pip
    pip install festo-edcon
4. Build Package
    ```bash
    colcon build --symlink-install
    ```



## Run

```bash
source install/setup.bash
ros2 run ros_festo_edcon edcon --ros-args -p edcon_config.host:=<ip address>
```
