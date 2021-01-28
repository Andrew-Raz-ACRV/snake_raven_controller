# Snake Raven Controller

This software is a ROS node that generates RAVEN joint position commands for it to follow a trajectory from keyboard teleoperation. There are 2 ROS nodes in this folder: talkerSnakeRaven and listenerSnakeRaven. An illustration of the control software flowchart is displayed below:

![alt text](https://github.com/Andrew-Raz-ACRV/snake_raven_controller/blob/master/ControlDiagram.PNG)

## ROS topics :
The two ROS nodes talkerSnakeRaven and listenerSnakeRaven exchange information through ROS topics. Below are the two ROS topics that this software use in order to communicate with the main RAVEN software. This is the [link](https://github.com/melodysu83/AutoCircle_generater/tree/master/msg) to the .msg files.

1. **raven_state.msg** : This topic stores current RAVEN state variable values. 
2. **raven_automove.msg** : This topic stores the motion command for RAVEN to move accordingly.


## ROS nodes :
1. **talkerSnakeRaven** : This ROS node is the command generator. It subscribes to raven_state.msg ROS topic to get current RAVEN status, then computes corresponding motion commands for the robot arm to follow circle trajectory. Finally, it publishes to raven_automove.msg ROS topic.
2. **listenerSnakeRaven** : This ROS node is only for testing. It serves the same purpose as the main RAVEN software, and will be replaced during actual use. It communicates with the talkerSnakeRaven by listening to the motion commands and sends out RAVEN state variable feedback after following the command.

## Prerequisite installation :
The snake_raven_controller uses Eigen to compute the kinematics control algorithms. To install this do:
1. Go to http://eigen.tuxfamily.org/index.php?title=Main_Page#Download to get the most recent Eigen. 
2. Download the zip, extract and find the subfolder "Eigen" 
3. In snake_raven_controller/ create the folder 'include' and paste 'Eigen' into the include folder.

## raven_2 main thread modifications :
The raven_2 source code was modified to with a new control mode that allowed incremental joint updates to from the 'raven_jointmove' topic.
These are the main modifications are in folder raven_2 inside subfolders /src /msg and /include

Important modifcations in raven_2/src

---**local_io.cpp** --------------------- This is where the new ros publisher and subscriber defined 'JointState' and 'raven_jointmove'

---**rt_raven.cpp** --------------------- This is where the new control mode is: 'raven_joint_velocity_control'

---**trajectory.cpp** ------------------- This is where the deltas increment the desired joint position: 'update_joint_position_trajectory'

In raven_2/include/raven

---**defines.h** ------------------------ Skip tool initialisation is enabled line 45: #define RICKS_TOOLS     //skips tool initialization

In raven_2/msg

----**raven_jointmove.msg**

----**raven_state.msg**

## snake_raven_controller main files : 
The source code for controlling snakeraven

**/msg folder:** 

----**raven_jointmove.msg**

----**raven_state.msg**

----**snakeraven_state.msg**
    
**/src folder:**

----**/raven_2 folder:**

--------**raven_state.h**

----**Raven_Controller.cpp** --------------------- (original) This is where the console interactions and modes are
 
----**Raven_Controller.h** ----------------------- (original) This class controls the threads and workflow.

----**SnakeRaven.cpp** -------------------- (original) This is where the kinematics functions are

----**SnakeRaven.h** ---------------------- (original) This class defines all the kinematic math functions

----**listener.cpp** ----------------------------- (original) A test node to replace the main RAVEN software.

----**talker.cpp** ------------------------------- (original) This file is where main is.

**CMakeLists.txt** ------------------------------- (original)

**README.md** ------------------------------------ (original)

**package.xml** ---------------------------------- (original) The ROS package.xml file.

The file talker.cpp is the heart of SnakeRaven controller, it is where the main is. This file uses on methods defined in class Raven_Controller. Inside of class Raven_Contoller, there are two threads - ros_thread and console_thread, which takes charge of the ROS publishing/subscribing issues and user console inputs respectively. The class Raven_Contoller depends on class SnakeRaven to compute and design snake robot trajectories. So, it is basically where all the math is. In the class Raven_Controller, there are two SnakeRaven objects managing the motion of LEFT and RIGHT arm of RAVEN. (All these files belong to the talkerSnakeRaven ROS node.)
If NOT actually connected to the main RAVEN software, the listener.cpp is the simplified version to simulate the main RAVEN software just so talker.cpp has someone to interact with. Thus, listener.cpp will be completely replaced when we actually combine it with the RAVEN code.


## How To Use This Code: 
This software design allows RAVEN to be controlled and follow predefined circular trajectory from any remote computer, so below is the instruction for hardware setup and terminal commands for it to work.
Computer A: The computer that downloads this snake_raven_controller software and runsroscore from here.
Computer B: This is the computer that RAVEN main software runs on. (This should be already setup and ready to go.)

1. **Find out hostname and IP** : On both computer A and computer B, open up a terminal, type in "hostname" to find out the hostname of that computer and type in "ifconfig" to find out the IP address. Note that it will be better and more stable connection-wise if both computers are connected to wired internet. Because wireless connections does not have static IP, and can cause floating IP problem which may cause us to lose connection midway through your control.
Up to this point, we should have: IP for computer A and B (denoted as IP_A and IP_B later on in the instruction),as well as the hostname for computer A and B  (denoted as hostname_A and hostname_B later on in the instruction).

2. **Download snake_raven_controller code** : On computer A, type in "git clone git://github.com/melodysu83/AutoCircle_generater.git" to download this repository to the directory you desire. It should create a AutoCircle_generater folder under that directory. Note that this should be a directory under the your ROS workspace. Then go back to your ROS workspace directory, type in "catkin_make_isolated --pkg AutoCircle_generater", and this should build the software. Now this part of the software is done.

3. **ssh** : On computer A, open up another terminal and type in "ssh -X hostname_A@IP_B", it should allow you to control computer B from this terminal. Assume you already have RAVEN running on this computer, type in "pwd", and then "roscd raven_2". Note that this RAVEN code should be the RAVEN 10.16 release version in October 2016, because there is a pedal down/pedal up switch where you can control by pressing 'd' and 'u' keys respectively. And this feature is required for the AutoCircle_generater to work. If you don't have the version yet, you can download it from [here](https://github.com/melodysu83/raven_2), and use the indigo branch.

4. **Up and running** : On computer A, open up another terminal and run "roscore". Use ths terminal is step 3, type in "sudo roslaunch raven_2 raven_2.launch". Next, initialize RAVEN. After it is initialized, press 'd' and it holds the pedal down state for 5 seconds. If no commend is sent within the time range, it will automatically switch back to pedal up state. If you are constantantly sending commends, then RAVEN will be in pedal down state all the time until you press 'u' manually. But you can always re-press 'd' again if you need to. Within the 5 second, use the terminal in step 2, type in "rosrun AutoCircle_genetater talkerAutoCircle" under the main ROS workspace directory, and you sill need to press "Enter" to start. At this point, you should see RAVEN moving slowly in very small circles already if everything is good. 


## Trouble Shooting : 
These are the problems that may occur to you. Basically if you are running AutoCircle_generater code, and still the pedal down state in RAVEN holds for 5 seconds then jumps back to pedal up, that means there is something wrong with the connection. Try doing the following to identify the problem . If you cannot even run the RAVEN code from terminal in step 2, then it is not even sensing the "roscore" on computer A.

1. **Internet Connection Issue** : (If you cannot run RAVEN code) On the terminal in step 2, where you ssh to computer B, type in "ping hostname_A", and make sure it works well. If not, try "ping IP_A". If hostname_A fails but IP_A works, that means computer B does not know the name of computer A. Do it by typing in "gedit /etc/hosts" and add in hostname_A IP_A to the list of hosts. If even "ping IP_A" does not work, then there is some internet issue, try checking your internet connection and double check the IP address. Do the same process for computer A on any other terminal, only to change by typing "ping hostname_B" and "ping IP_B". If both sides are working good, 

2. **ROS_MASTER_URL** : Make sure you set the ROS_MASTER_URL correctly. It should be the following: On regular terminal on computer A, type in "gedit ~/.bashrc", then set ROS_MASTER_URL = host_A (or IP_A), ROS_IP = host_A (or IP_A). On the terminal you ssh to computer B, type in "gedit ~/.bashrc", then set ROS_MASTER_URL = host_A (or IP_A), ROS_IP = host_B (or IP_B). This can be a tricky part, so be sure to set it correctly. This should ensure the ROS topic publishing and subscribing to be bidirectional.

3. **rostopic monitoring** : You can type in "rostoppic echo topicname -c" this on each side to monitor the publishing and subscribing condition and check if it's the problem of receiving ravenstate or sending raven_automove.


## Selection Menu : 
User can choose to control joints individually or control the desired end-effector pose in keyboard teleoperation.


## Spec : 

1. **publish rate** : The raven_jointmove.msg is being sent at 1000 Hz.

2. **feedback rate** : The raven_state.msg is being sent at 100 Hz in listener.cpp. But in actual RAVEN software, raven_state.msg is updated at 1000 Hz.


## Relavent links:
1. **uw-biorobotics/raven2** : This is the main RAVEN software to connect to. And [this code](https://github.com/uw-biorobotics/raven2) will replace the ROS node listenersnakeraven that we temporarily have for now.
2. **AutoCircle_generator** : This code is based on the AutoCircle_generator code https://github.com/melodysu83/AutoCircle_generater
