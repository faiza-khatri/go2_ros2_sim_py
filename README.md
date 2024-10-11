## Simulation go1 quadropted robot in gazebo Ignition

### This builded and tested on ROS2 HUMBLE


### install dependenses:



### SETUP:

```bash
mkdir -p go1_sim_py/src
cd go1_sim_py/src
git clone https://github.com/abutalipovvv/go1_sim_py.git .
cd ..
colcon build
```

### RUN SIMULATION

```bash
cd go1_sim_py
source install/local_setup.bash
ros2 launch go1_description go1_gazebo.launch.py
```


### move with teleop_twist_keyboard:

```bash
cd go1_sim_py
source install/local_setup.bash
ros2 launch go1_description go1_gazebo.launch.py
```

```bash
source install/local_setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```