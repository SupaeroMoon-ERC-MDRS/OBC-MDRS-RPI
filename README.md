# OBC-MDRS-RPI

This repository will house the main files to go on the raspberry pi for the MDRS mission. This repository does not include any ROS packages or distributions and all dependencies will need to be installed prior to use.
## Docker setup for Raspberry pi
After [installing Docker](https://docs.docker.com/engine/install/raspberry-pi-os/) on the board, transfer the files in directory ```/src/rpi_MDRS/Docker``` in the local file system of the board.

NOTE: It is required to know set up the connection and know IP address of the Raspberry for the following step.

Use *scp* on your local machine to transfer files towards the board, using the command:


```bash
scp -r OBC-RPI-MDRS/src/rpi_MDRS/Docker supaeromoon@<rasp-ip>:/
```

Then build and run the container having the container terminal available, by using the file ```docker-compose.yml``` that calls ```Dockerfile``` to build the container in the first place, using the command: 

```bash
cd Docker
docker compose up -d
```

For development on architectures different than **arm64** edit the ```docker-compose.yml``` file, dependent on the host platform.

Moreover is possible to run the container through [Visual Studio Code Dev Containers](https://www.youtube.com/watch?v=dihfA7Ol6Mw), having the container environment identical to the local environment.


## ROS Launch
Installation:
This repository is intended for use with ROS2 Humble.
To install dependencies, run
```bash
rosdep install --from-paths . --ignore-src -r -y src/sim_mdrs/config/dependencies.yaml --rosdistro humble
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