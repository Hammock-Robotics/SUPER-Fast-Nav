To run the simulation:
```bash
GAZEBO_MODEL_PATH=$(pwd)/src/iq_sim/models/ roslaunch iq_sim dash_runway.launch
```


## Important -- when launching the physical drone

Change the address of the UDP port for SITL/physical drone

Change imu, extrinsic T in mid360 yaml

Change the click height in the click_demo config in order to publish variable z heights

Change the eth0 ip address to match that of the mid360 (or any LIDAR used) eg 192.168.1.50
