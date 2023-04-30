# Robothon 2023 Team Platonics

## Installation
### [Franka Cartesian Impedence Controller](https://github.com/platonics-delft/franka_control_robothon_challenge)
First, we need to install *libfranka* to communicate with the robot. The library is publicly available and can be installed in the `$HOME` directory using the following commands.

```
cd $HOME
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
git clone --recursive https://github.com/frankaemika/libfranka
cd libfranka
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```

In the next step, we install the ROS-wrapper (we assume that you have set up a real-time kernel with ROS) for the communication with the robot.

```
cd $HOME
mkdir -p catkin_ws/src
cd catkin_ws
source /opt/ros/<ros-distro>/setup.sh
catkin_init_workspace src
git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros
rosdep install --from-paths src --ignore-src --rosdistro <ros-distro> -y --skip-keys libfranka
source devel/setup.sh
```

Finally, we can install the custom controller used by the Platonics in the Robothon 2023 using the following:

```
roscd franka_ros
git clone git@github.com:platonics-delft/franka_control_robothon_challenge.git
roscd
cd ..
source /opt/ros/<ros-distro>/setup.bash
catkin_make -DMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/libfranka/build
```

You can now run the controller using

```
roslaunch franka_human_friendly_controllers cartesian_variable_impedance_controller.launch robot_ip:=ROBOT_IP load_gripper:=True
```

### [robothon23_vision](https://github.com/platonics-delft/robothon23_vision)
The vision components are a pure python package and can be installed using `pip`. This will automatically install the dependencies, i.e. `opencv-python`, `scipy`, `numpy`.

```
git clone git@github.com:platonics-delft/robothon23_vision.git
cd robothon23_vision
pip install .
```

### [robothon23_gui](https://github.com/platonics-delft/robothon23_gui)
The GUI can be installed using:

```
git clone git@github.com:platonics-delft/robothon23_gui.git
cd robothon23_gui
pip install .
```

### [ROS packages](https://github.com/platonics-delft/robotthon_23_platonics)
Finally, all the ROS-related packages are located here. For example, the trajectory manager and the active box_localization.

```
sudo apt install ros-<ros-distro>-realsense-ros
cd $HOME/catkin_ws
source devel/setup.bash
cd src
git clone git@github.com:platonics-delft/robotthon_23_platonics.git#debug-camera-feedback
roscd
cd ..
catkin_make
source devel/setup.bash
```

## Getting started: Demonstrate, Correct, Generalize, Execute

First of all we need to start the controller with

```
roslaunch frank_human_friendly_controllers cartesian_variable_impedance_controller.launch robot_ip:=ROBOT_IP
```

Then we start the camera with

```
roslaunch trajectory_manager controller_camera.launch 
```

You can now record a demonstration with:

```
python3 manipulation_tasks_panda/trajectory_manager/recording_trajectory.py "name skill"
```

For executing the skill you can run 

```
python3 manipulation_tasks_panda/trajectory_manager/playback_trajectory.py "name skill"
```

To run all the skill in a row, you can modify the `manipulation_tasks_panda/trajectory_manager/playall.py` according to the name of your skills. The you can run

```
python3 manipulation_tasks_panda/trajectory_manager/playall.py 0
```

if you don't want to have the active localizer or

```
python3 manipulation_tasks_panda/trajectory_manager/playall.py 1
```

otherwise.

The second command requires that you also launch the localization service with

```
roslaunch box_localization box_localization.launch 
```

In this second case, the robot will first localize the object to match the template given during demonstration, trasform the skill in the new reference frame and then execute it. 

During demonstration or during execution, it is possible to give feedback to the learned skill using the computer keyboard.

- Press `e` to stop the recording

**Gripper commands:**

- Press `c` to close the gripper 
- Press `o` to open the gripper 

**Camera feedback:**

- Press `k` to add a label to enable the camera feedback from that point on 
- Press `l` to add a label to disable the camera feedback from that point on 

**Haptic feedback:**

- Press `z` to add a label to enable the haptic feedback from that point on 
- Press `x` to add a label to disable the haptic feedback from that point on 

The motivation of explicitly labeling or disabling the haptics and local camera feedback is because during a long trajectory the user can explicitly teach the robot to use or not that skill. For example, it makes sense to have the haptic feedback only active when doing insertion tasks, such that the robot will not start spiraling when external forces are perceived but they are not due to critical insertion tasks. It is worth noticing, that if no insertion task is performed, in case of large force, the robot would temporarily stop the execution of the motion, until the disturbance is gone.

**Directional feedback:**

- Press `w` to locally slightly shift the trajectory in positive x in robot frame 
- Press `s` to locally slightly shift the trajectory in negative x in robot frame
- Press `a` to locally slightly shift the trajectory in positive y in robot frame
- Press `d` to locally slightly shift the trajectory in negative y in robot frame
- Press `u` to locally slightly shift the trajectory in positive z in robot frame
- Press `j` to locally slightly shift the trajectory in negative z in robot frame

This feedback is useful when, after the demonstration, the robot will, for example, not press the button hard enough. However, by pressing `j`, the trajectory is locally modified and when executing again, the robot will apply enough force.

