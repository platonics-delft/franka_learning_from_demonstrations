# Software Overview
Our solution to the robothon 2023 challenge is made fully open-source to allow the community to build on top and further improve our solution. All components are stored in [Platonics Delft](https://github.com/orgs/platonics-delft). You can find all required software components and links to their installation guides below.

1. [Franka Cartesian Impedence Controller](https://github.com/platonics-delft/franka_control_robothon_challenge)
2. [robothon23_vision](https://github.com/platonics-delft/robothon23_vision)
3. [robothon23_gui](https://github.com/platonics-delft/robothon23_gui)
4. [robothon23_manipulation](https://github.com/platonics-delft/robothon23_manipulation)

## Installation

### Franka Cartesian Impedence Controller
First, we need to install `libfranka` to communicate with the robot. The library is publicly available and can be installed in the `$HOME` directory using the following commands:

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

In the next step, we install the ROS-wrapper (we assume that you have set up a real-time kernel with ROS) for the communication with the robot:

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

Finally, we can install the custom controller used by the Platonics in the robothon 2023 using the following:

```
roscd franka_ros
git clone git@github.com:platonics-delft/franka_control_robothon_challenge.git
roscd
cd ..
source /opt/ros/<ros-distro>/setup.bash
catkin_make -DMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/libfranka/build
```

You can now run the controller using:

```
roslaunch franka_human_friendly_controllers cartesian_variable_impedance_controller.launch robot_ip:=ROBOT_IP load_gripper:=True
```

### robothon23_vision
The vision components are a pure Python package and can be installed using `pip`. This will automatically install the dependencies, i.e. `opencv-python`, `scipy`, `numpy`.

```
git clone git@github.com:platonics-delft/robothon23_vision.git
cd robothon23_vision
pip install .
```

### robothon23_gui
The GUI can be installed using:

```
git clone git@github.com:platonics-delft/robothon23_gui.git
cd robothon23_gui
pip install .
```
### robothon23_manipulation

Finally, all the ros related packages are located here. For example, the trajectory manager and the active box_localization.

```
sudo apt install ros-<ros-distro>-realsense-ros
cd $HOME/catkin_ws
source devel/setup.bash
cd src
git clone git@github.com:platonics-delft/robothon23_manipulation.git
pip install -r robothon23_manipulation/requirements.txt
cd ..
catkin_make
source devel/setup.bash
```

## Getting started

### Send the robot in home position 

```bash
roslaunch franka_example_controllers move_to_start.launch robot_ip:=ROBOT_IP
```

### Record the template of the object in the current robot view 

To record and save the template that is going to be used later to localize the object follow the instructions: [here](https://github.com/platonics-delft/robothon23_gui/blob/main/README.md)

### Kinesthetic Demonstration 

First of all we need to start the controller with

```bash
roslaunch franka_human_friendly_controllers cartesian_variable_impedance_controller.launch robot_ip:=ROBOT_IP
```

Then we start the camera with

```bash
roslaunch box_localization camera.launch 
```

You can now record a demonstration with:

```bash
roscd trajectory_manager/scripts
./recording_trajectory "name of skill"
```

During demonstration the robot is recording the Cartesian pose of the robot and the current image from the camera for each time step (the control freqeuncy is set to 20 Hz).
During execution, the robot tracks the recorded trajectory while trying to correct for the discrepancy between the current image at that timestep and the one recorded during demonstration.
This increases the reliability of critical picking and insertion tasks. 

### Execute Learned Skill 

For executing the skill you can run 

```bash
roscd trajectory_manager/scripts
./playback_trajectory "name of skill"
```

To run all the skills in a row, you can modify the `play_all` according to the names of your skills. If you don't want this script to use 
the active localizer you can then run 

```bash
roscd trajectory_manager/scripts
./play_all 0
```

If you instead do want to use the active localizer, you can then run

```bash
roscd trajectory_manager/scripts
./play_all 1
```

This second option requires that you also launch the localization services with 

```bash
roslaunch box_localization box_localization.launch 
```

In this second case, the robot will first localize the object by trying to match the template given during demonstration, and then trasform the skill(s) in the new reference frame before executing them. 

### Give Corrections to the recorded Demonstration 

During demonstration or during execution, it is possible to give feedback to the learned skill using the computer keyboard. 

- Press `e` to stop the recording.

#### Gripper commands:

- Press `c` to close the gripper.
- Press `o` to open the gripper.

#### Camera feedback:

- Press `k` to add a label to enable the local camera feedback from that point on.
- Press `l` to add a label to disable the local camera feedback from that point on.

#### Haptic feedback:

**Haptic feedback:**

- Press `z` to add a label to enable the haptic feedback from that point on
- Press `x` to add a label to disable the haptic feedback from that point on

The motivation for explicitly labeling or disabling the haptic and local camera feedback is that during a long trajectory, the user can explicitly teach the robot to use or not use that skill. For example, it makes sense to have the haptic feedback only active when performing insertion tasks, so that the robot will not start spiraling when external forces are perceived but they are not due to critical insertion tasks. It is worth noticing that if no insertion task is performed, in case of large force, the robot would temporarily stop the execution of the motion until the disturbance is gone. This increases safety when interacting with humans.

**Directional feedback:**

- Press `w` to locally shift the trajectory in positive x in robot frame 
- Press `s` to locally shift the trajectory in negative x in robot frame 
- Press `a` to locally shift the trajectory in positive y in robot frame 
- Press `d` to locally shift the trajectory in negative y in robot frame 
- Press `u` to locally shift the trajectory in positive z in robot frame 
- Press `j` to locally shift the trajectory in negative z in robot frame 

This feedback is useful when, after the demonstration, the robot will, for example, not press the button hard enough. However, by pressing `j`, the trajectory is locally modified, and when executing again, the robot will apply enough force.

At the end of every play back, the computer will ask if to overwrite the old trajectory or not. If the changes are accepted, the robot will remember the feedback in the next executions.
