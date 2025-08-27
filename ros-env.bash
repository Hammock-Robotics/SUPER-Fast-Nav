source /opt/ros/noetic/setup.bash 
source devel/setup.bash
source /usr/share/gazebo/setup.sh

GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/dash/SUPER-Fast-Nav/src/iq_sim/models:/home/dash/SUPER-FAST-Nav/3rdparty/ardupilot_gazebo/models
GAZEBO_PLUGIN_PATH=$GAZEBO_MODEL_PATH:/home/dash/SUPER-Fast-Nav/3rdparty/ardupilot_gazebo/build
