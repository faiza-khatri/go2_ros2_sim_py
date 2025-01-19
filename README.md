## Simulation quadropted robots in gazebo sim

### This builded and tested on ROS2 JAZZY


### install dependenses:



### SETUP:

```bash
mkdir -p ~/go_sim/src
cd ~/go_sim/src
git clone https://github.com/abutalipovvv/go1_sim.git .
cd ..
colcon build --symlink-install
```

### install deps:
```bash
cd ~/go_sim
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### RUN SIMULATION

```bash
cd ~/go_sim
source install/local_setup.bash
ros2 launch gazebo_sim launch.py
```


### move with teleop_twist_keyboard:
```bash
source install/local_setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/<robot_namespace>/cmd_vel
```


## Credits

    mike4192: https://github.com/mike4192/spotMicro
    Unitree Robotics: https://github.com/unitreerobotics/a1_ros
    QUADRUPED ROBOTICS: https://quadruped.de
    lnotspotl : https://github.com/lnotspotl
    anujjain-dev : https://github.com/anujjain-dev/unitree-go2-ros2

## TODO:
    gazebo classic (physic inertions for urdf)
    odometry calibration (now is working with only cmd_vel commands for dx,dy)
       we need forward kinematics solution, or not?
