## Simulation go1 quadropted robot in gazebo Ignition

### This builded and tested on ROS2 HUMBLE


### install dependenses:



### SETUP:

```bash
mkdir -p go1_sim_py/src
cd go1_sim_py/src
git clone https://github.com/abutalipovvv/go1_sim.git .
cd ..
colcon build
```

### install deps:
```bash
rosdep update
cd go1_sim_py
rosdep install --from-paths src --ignore-src -r -y
```

### RUN SIMULATION

```bash
cd go1_sim_py
source install/local_setup.bash
ros2 launch go1_description go1_gazebo.launch.py
```


### move with teleop_twist_keyboard:
```bash
source install/local_setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


## Credits

    mike4192: https://github.com/mike4192/spotMicro
    Unitree Robotics: https://github.com/unitreerobotics/a1_ros
    QUADRUPED ROBOTICS: https://quadruped.de
    lnotspotl : https://github.com/lnotspotl

## TODO:
    controller calibration
    gazebo classic
    add namespaces
    multi robots
    nav2: omni, mppi
    go2_desctiption
    create interface for robot control
    aruco detection with camera
