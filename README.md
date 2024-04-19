# Software Overview
Our solution to the robothon 2023 challenge is made fully open-source to allow the community to build on top and further improve our solution. All components are stored in [Platonics Delft](https://github.com/orgs/platonics-delft). You can find all required software components and links to their installation guides below.

1. [Franka Cartesian Impedence Controller](https://github.com/platonics-delft/franka_impedance_controller)
2. [Learning from Demonstrations](https://github.com/platonics-delft/franka_learning_from_demonstrations)

### Install the controller on the computer connected to the robot 
Follow the instructions here:
https://github.com/platonics-delft/franka_learning_from_demonstrations

### Install realsense camera and calibrate the extrinsic parameters (hand-eye calibration)
Follow the instructions here:
https://github.com/franzesegiovanni/franka_easy_handeye


Be sure to add a static transfor of the camera frame with respect to the robot hand frame in object_localization/config/camera_transforms.yaml.    
The parameters for position and orientation are coming from the calibration procedure. 


## Installation of the python controller to perform learning from demonstration and object localization. You can install this also on another computer connected to the same network as the robot. 

```
mkdir robot_ws
cd robot_ws
mkdir src
cd src
git clone --depth 1 https://github.com/platonics-delft/franka_learning_from_demonstrations
git clone https://github.com/platonics-delft/panda-ros-py.git
git clone https://github.com/franzesegiovanni/quaternion_algebra.git
git clone --depth 1 https://github.com/platonics-delft/trajectory_data.git
pip install -r robothon23_manipulation/requirements.txt
cd ..
catkin build
source devel/setup.bash
```
Please remember to source the workspace in every terminal you open.

## Let's start to learn some skills! 

### Start the controller on the computer connected to the robot 

``` bash 
roslaunch franka_robothon_controllers cartesian_variable_impedance_controller.launch robot_ip:=ROBOT_IP
```

### Send the robot to the home position and record the current template for the localization 

Send the robot to the home position. The robot will move in front of the robot and we can specify the z axis, i.e. the robot vertical height as one of the input to the script. For example, to send the robot to the home position at 0.25 m from the table, run: 
``` bash
    roslaunch trajectory_manager home.launch height:="0.25" 
```

Record the current template for the localization 
``` bash
    roslaunch object_localization record_template.launch template_name:='template'
```
### Kinesthetic Demonstration 

Be sure that the camera is running using: 

```bash
roslaunch box_localization camera.launch
```

You can now record a demonstration with:

```bash
roslaunch skills_manager record_skill.launch name_skill:='skill'
```
All the trajectories are saved in the folder `trajectory_data/trajectories/` with the name you gave to the skill.
This folder is a ros package that is used to save and load the demonstrations and save them. We used this folder to have a ligher repository and save all the demonstration in this other one. 

During demonstration the robot is recording the Cartesian pose of the robot and the current image from the camera for each time step (the control freqeuncy is set to 20 Hz).
During execution, the robot tracks the recorded trajectory while trying to correct for the discrepancy between the current image at that timestep and the one recorded during demonstration.
This increases the reliability of critical picking and insertion tasks. 

### Execute Learned Skill 

For executing the skill you can run 

```bash
roslaunch skills_manager play_skill.launch localize_box:=true name_skill:='skill'

```

Or you can execute more skills 
```bash
roslaunch skills_manager play_all_skills.launch localize_box:=true 
```


In this second case, the robot will first localize the object by trying to match the template given during demonstration, and then trasform the skill(s) in the new reference frame before executing them. 

### Commands during demonstration
During demonstration or during execution, it is possible to give feedback to the learned skill using the computer keyboard. 

- Press `e` to stop the recording.

### Pause the demonstration 
- Press 'Space tab' to pause/start the demonstration. Be carefull that when you start again you are close to a point you where when you paused it

#### Gripper commands:

- Press `c` to close the gripper.
- Press `o` to open the gripper.

#### Camera feedback:

- Press `k` to add a label to enable the local camera feedback from that point on.
- Press `l` to add a label to disable the local camera feedback from that point on.

#### Haptic feedback:

- Press `z` to add a label to enable the haptic feedback from that point on
- Press `x` to add a label to disable the haptic feedback from that point on

#### Stiffen the orientation of the end-effector:

- Press `m` makes the robot stiffen the orientation of the end-effector in the current configuration
- Press `n` makes the robot orientation stiffness to be null again


The motivation for explicitly labeling or disabling the haptic and local camera feedback is that during a long trajectory, the user can explicitly teach the robot to use or not use that skill. For example, it makes sense to have the haptic feedback only active when performing insertion tasks, so that the robot will not start spiraling when external forces are perceived but they are not due to critical insertion tasks. It is worth noticing that if no insertion task is performed, in case of large force, the robot would temporarily stop the execution of the motion until the disturbance is gone. This increases safety when interacting with humans.

### Give interactive Corrections during execution when running each skill separately and without the localizer 
```bash
roslaunch skills_manager play_skill.launch localize_box:=false name_skill:='skill'
```

#### Directional feedback:

- Press `w` to locally shift the trajectory in positive x in robot frame 
- Press `s` to locally shift the trajectory in negative x in robot frame 
- Press `a` to locally shift the trajectory in positive y in robot frame 
- Press `d` to locally shift the trajectory in negative y in robot frame 
- Press `u` to locally shift the trajectory in positive z in robot frame 
- Press `j` to locally shift the trajectory in negative z in robot frame 

This feedback is useful when, after the demonstration, the robot will, for example, not press the button hard enough. However, by pressing `j`, the trajectory is locally modified, and when executing again, the robot will apply enough force.

### Speed feedback:
You can make the robot to go faster locally by 
- Press `f` to lcoally make the motion faster. 
 
 This interactive feedback can be used to make the robot faster during the execution of the motion. 

You can also correct when the gripper has to close or if you want to active/disactivate the haptic feedback or the camera feedback, see previous Section. 

At the end of every play back, the computer will ask if to overwrite the old trajectory or not. If the changes are accepted, the robot will remember the feedback in the next executions.

That's it! Have fun!

If you encounter any issue, please send an email to: g.franzese@tudelft.nl
