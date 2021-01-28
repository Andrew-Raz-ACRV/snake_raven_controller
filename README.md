# SnakeRaven a 3D printed Snake-like manipulator for the Raven II
SnakeRaven is an instrument that can be attached to the Raven II and controlled via keyboard teleoperation. 

This software primarily contains:
- ROS nodes for controlling SnakeRaven when attached to the Raven II
- Modifications to the raven_2 source code to have a new velocity joint control mode
- A class with functions that solves the forward and inverse kinematics of SnakeRaven in C++

![alt text](https://github.com/Andrew-Raz-ACRV/snake_raven_controller/blob/master/FrontCoverSnake2.png)

A. SnakeRaven on the Raven II 

B. The adaptor piece

C. The snake endeffector 

# Snake Raven Controller

This software contains a ROS node that generates joint position commands for Raven to follow a trajectory from keyboard teleoperation. There are 2 ROS nodes in this folder: talkerSnakeRaven (the main node) and listenerSnakeRaven (a test node that can be used if not connected to the Raven II). 

The Raven II main software **raven_2** is modified (these modifications are in folder [raven_2](https://github.com/Andrew-Raz-ACRV/snake_raven_controller/tree/master/raven_2)) to comunicate with the **snake_raven_controller** via publishers and subscribers. An illustration of the control software flowchart is displayed below:

![alt text](https://github.com/Andrew-Raz-ACRV/snake_raven_controller/blob/master/ControlDiagram.PNG)

## ROS Node-topic communication :
The two ROS nodes talkerSnakeRaven and r2_control (or listenerSnakeRaven) exchange information through ROS topics. The ROS node communication is shown in the rqt_graph below:

![alt text](https://github.com/Andrew-Raz-ACRV/snake_raven_controller/blob/master/rqt_graph_snakeraven.png)

1. **talkerSnakeRaven** : rosrun this file to run node **/snake_raven_controller** It subscribes to topic **/joint_states** to get the current joint values and publishes joint deltas in topic **/raven_jointmove**
2. **listenerSnakeRaven** : This ROS node is only for testing. It can be used to replace the actual node **/r2_control**

## Prerequisite installation :
The snake_raven_controller uses Eigen to compute the kinematics control algorithms. To install this do:
1. Go to http://eigen.tuxfamily.org/index.php?title=Main_Page#Download to get the most recent Eigen. 
2. Download the zip, extract and find the subfolder "Eigen" 
3. In snake_raven_controller/ create the folder 'include' and paste 'Eigen' into the include folder.

## raven_2 main thread modifications :
The raven_2 source code was modified to have a new control mode that allowed incremental joint updates from the 'raven_jointmove' topic.
These are the main modifications are in folder raven_2 inside subfolders /src /msg and /include

Important modifcations in raven_2/src

---**local_io.cpp** --------------------- This is where the new ros publisher and subscriber defined 'JointState' and 'raven_jointmove'

---**rt_raven.cpp** --------------------- This is where the new control mode is: 'raven_joint_velocity_control'

---**trajectory.cpp** ------------------- This is where the deltas increment the desired joint position: 'update_joint_position_trajectory'

In raven_2/include/raven

---**defines.h** ------------------------ Skip tool initialisation is enabled line 45: #define RICKS_TOOLS     //skips tool initialization

In raven_2/msg

----**raven_jointmove.msg** ------------- This is where the joint deltas come through

## snake_raven_controller main files : 
The source code for controlling snakeraven

**/src folder:**

----**talker.cpp** ---------------------- This file is where main is and it calls the Raven_Controller class

----**Raven_Controller.cpp** ------------ This is where the console interactions and modes are
 
----**Raven_Controller.h** -------------- This class controls the threads and workflow.

----**SnakeRaven.cpp** ------------------ This is where the kinematics functions are for SnakeRaven

----**SnakeRaven.h** -------------------- This class defines all the SnakeRaven functions

----**listener.cpp** -------------------- A test node to replace the main RAVEN software.

**/msg folder:** 

----**raven_jointmove.msg**  ------------ This is where the joint deltas come through

**CMakeLists.txt** ----------------------

**README.md** ---------------------------

**package.xml** ------------------------- The ROS package.xml file.

The file talker.cpp is the heart of SnakeRaven controller, it is where the main is. This file uses the class Raven_Controller. Inside Raven_Contoller, there are two threads - ros_thread and console_thread, which takes charge of the ROS publishing/subscribing issues and user console inputs respectively. The class Raven_Contoller depends on class SnakeRaven to compute and design snake robot trajectories.

## How To Use This Code on the Raven II: 
If you have an assmbled SnakeRaven tool you can follow these instuctions to integrate it to the Raven II with this code:

1. **Download** : On the Raven II computer download this package and place the folder **snake_raven_controller** in the Raven II folder e.g. home/raven_18_05

2. **Update** : Go to raven_18_05/raven_2 and update its contents with the three /src /msg/ includes folders in the contents of folder [raven_2](https://github.com/Andrew-Raz-ACRV/snake_raven_controller/tree/master/raven_2)

3. **Make** : Open a new terminal and cd to the Raven II directory. Run catkin_make to compile the new content and modifications
```
cd raven_18_05
source devel/setup.bash
catkin_make
```

4. **Run the Raven II** : Without any instruments on the Raven II, roslaunch the robot and press the e-stop and release to go through homing. 
```
roslaunch raven_2 raven_2.launch
```

5. **Velocity Joint Control mode** : Press 'm' to change mode and press '2' to start the velocity joint control mode, press the e-stop and release for the mode change to occur.

6. **Run talkerSnakeRaven** : Open a new terminal and run command:
```
rosrun snake_raven_controller talkersnakeraven
```

7. **Selection Menu** : At this point you can choose either:

_0. **Calibration** - this moves the right arm to be perpendicular to the table and the left arm aside. It will pop up a message saying that you can insert the SnakeRaven tool onto the tool holder. You can use the keyboard to adjust the joints of the robot inividually particularly to insert the SnakeRaven tool to mesh with the robot. Good Calibration is when SnakeRaven is neutral and perpendicular to the table as seen in the SnakeRaven [image](https://github.com/Andrew-Raz-ACRV/snake_raven_controller/blob/master/FrontCoverSnake2.png).

_1. **Joint Control** - this allows you to control the robot joints individually with the keyboard without calibration

**Joint Control Keyboard Mapping:**

Left Arm
- Shoulder +/-:      1/q
- Elbow +/-:         2/w
- Z Insertion +/-:   3/e
- Tool Rotation +/-: 4/r
- Wrist +/-:         5/t
- Grasp 1 +/-:       6/y
- Grasp 2 +/-:       7/u

Right Arm
- Shoulder +/-:      a/z
- Elbow +/-:         s/x
- Z Insertion +/-:   d/c
- Tool Rotation +/-: f/v
- Wrist +/-:         g/b
- Grasp 1 +/-:       h/n
- Grasp 2 +/-:       j/m

_2. **Teleoperation** - this allows you to control the SnakeRaven endeffector via keyboard but only after calibration. It has some additional code in Raven_Controller.cpp to:
- record teleoperation control onto a .csv file in the home folder
- Test the calibration by going to different joint poses at timed intervals

**Endeffector Keyboard Mapping:**
Relative to the Remote Centre of Motion
- Z+     q
- Z-     e
- Y+     w
- Y-     s
- X+     d
- X-     a
- freeze z
- pan +  f
- pan -  h
- tilt + t
- tilt - g
- z_rot+ y
- z_rot- r

8. **Shut down** : Press the e-stop and use 'k' to return to the selection menu and use ctrl-c in both terminals to stop the programs.

## Node comunication rate : 

1. **publish rate** : The raven_jointmove.msg is being sent at 1000 Hz.

2. **feedback rate** : The joint_state is being sent at 100 Hz in listener.cpp. But in actual RAVEN software, raven_state.msg is updated at 1000 Hz.

## Contact

This code is written by Andrew Razjigaev. If there are queries you can contact him via email: andrew.razjigaev@hdr.qut.edu.au

## Relavent links:
1. **uw-biorobotics/raven2** : This is the main RAVEN software to connect to. And [this code](https://github.com/uw-biorobotics/raven2) will replace the ROS node listenersnakeraven that we temporarily have for now.
2. **AutoCircle_generator** : This code is based on the [AutoCircle_generator code](https://github.com/melodysu83/AutoCircle_generater)
