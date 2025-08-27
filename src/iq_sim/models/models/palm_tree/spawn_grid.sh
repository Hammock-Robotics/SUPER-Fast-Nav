#!/bin/bash
# spawn_trees.sh

# 1) load ROS/Gazebo env
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# 2) wait for Gazebo to be ready
rosservice wait /gazebo/spawn_sdf_model

# 3) path to your tree model SDF
MODEL_SDF="${HOME}/.gazebo/models/my_mesh_model/model.sdf"

# 4) loop to spawn a 5×5 grid, spacing 9 m
for i in {0..4}; do
  for j in {0..4}; do
    X=$(echo "$i * 9.0" | bc)   # 0, 9, 18, 27, 36
    Y=$(echo "$j * 9.0" | bc)   # 0, 9, 18, 27, 36
    rosrun gazebo_ros spawn_model \
      -file "$MODEL_SDF" \
      -sdf \
      -model "palm_tree_${i}_${j}" \
      -x $X -y $Y -z 0
  done
done


