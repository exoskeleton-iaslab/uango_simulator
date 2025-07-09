# Uango ROS-Gazebo Simulation  

### Environment Dependence

We need ROS-Gazebo interface and controller packages:

```
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-ros-controllers
```

### Install the Package

```
cd ~/catkin_ws/src/
git clone https://github.com/exoskeleton-iaslab/uango_simulator.git
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

### Launch the Simulation

Launch the exoskeleton model:

```
roslaunch exoskeleton world.launch
```

Or, to filter out the wrench messages:

```
roslaunch exoskeleton world.launch | grep -v "ApplyBodyWrench"
```

# Configuration file

Launch the `rqt_gui` for control and tuning parameter:

```
roslaunch exoskeleton rqt.launch
```

Upper half for publish joint command.
Lower half for reconfigure joint controller PID parameters.
(Default is {p: 100, i: 0.01, d: 10} configured in `config/control.yaml`)

### ROS Topic 

Totally seven links:

`waist_link` `LU_link` `RU_link` `LD_link` `RD_link` `LF_link` `RF_link`

Totally six joints:

 `LU_joint` `RU_joint` `LD_joint` `RD_joint` `LF_joint` `RF_joint`

*L for left, R for right, U for up, D for down, F for foot.*

*e.g. LU_link > left up link (left thigh)*

Joint position controller:

```
/exoskeleton/LU_position_controller/command
```

### Gazebo Tips  
It may be useful to press `space` to pause the simulation and `ctrl-r` to reset world. 
