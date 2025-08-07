# Catkin Workspace

## Packages

The following packages were installed according to the Intelligent Quad tutorials at https://github.com/Intelligent-Quads/iq_tutorials

- `iq_sim` — [Contains most of our launch files and scripts for communication between Fast-Lio, SUPER and Mavros]
- `mavlink` — [description]
- `mavros`  — [description]
- `Mid360_simulation_plugin` — [Download from https://github.com/fratopa/Mid360_simulation_plugin]

## Usage

```
cd ~/catkin_ws
catkin build
source devel/setup.bash
roslaunch iq_sim dash_runway.launch
```
