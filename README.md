# OBC-MDRS-RPI

This repository will house the main files to go on the raspberry pi for the MDRS mission. This repository does not include any ROS packages or distributions and all dependencies will need to be installed prior to use.


Installation:
This repository is intended for use with ROS2 Humble.
To install dependencies, run rosdep install --from-paths . --ignore-src -r -y -f src/rpi_MDRS/config/dependencies.yaml


Do this later when setting up the rpi :
Setting up correct names and permissions for USB ports [Only perform this on RPi]

Make sure to follow all the steps described in USB Port name configuration section here — https://github.com/Ekumen-OS/andino/tree/humble/andino_hardware#usb-port-name-configuration