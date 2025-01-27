# OBC-MDRS-RPI

This repository will house the main files to go on the raspberry pi for the MDRS mission. This repository does not include any ROS packages or distributions and all dependencies will need to be installed prior to use.


## ROS Launch
Installation:
This repository is intended for use with ROS2 Humble.
To install dependencies, run
```bash
sudo apt update
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y

```



Simulation:
from home directory:
```bash
colcon build
source install/setup.bash
ros2 launch sim_mdrs mdrs.launch.py
```

To drive around, open a new terminal (leave gazebo running) and run:
```bash
ros2 run sim_mdrs keyboard_control
```

Do this later when setting up the rpi :
Setting up correct names and permissions for USB ports [Only perform this on RPi]

Make sure to follow all the steps described in USB Port name configuration section here — https://github.com/Ekumen-OS/andino/tree/humble/andino_hardware#usb-port-name-configuration