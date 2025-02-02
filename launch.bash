# Use by typing: bash launch.bash
# Description: This script is used to test the remote control to wheels 
# launching ros with 'roboclaw_testing.launch.py'

sudo docker start –ai 8789 
source /opt/ros/humble/setup.bash 
cd OBC-MDRS-RPI 
git pull 
colcon build 
source install/setup.bash 
ros2 launch sim_mdrs roboclaw_testing.launch.py 