#ifndef RAVEN_CONTROLLER_H_
#define RAVEN_CONTROLLER_H_

//#include "Raven_PathPlanner.h"
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <iomanip>
#include <ros/ros.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <sstream>
#include <pthread.h>
#include <termios.h>
#include <queue>
//my laptop
#include "/home/andrew/catkin_ws/src/snake_raven_controller/src/raven_2/raven_automove.h"
#include "/home/andrew/catkin_ws/src/snake_raven_controller/src/raven_2/raven_state.h"

//on the raven computer
//#include "/home/qut-raven/raven_18_05/src/snake_raven_controller/src/raven_2/raven_automove.h"
//#include "/home/qut-raven/raven_18_05/src/snake_raven_controller/src/raven_2/raven_state.h"

//Snake Raven Class
#include "SnakeRaven.h" // Snake Raven Class

//Arm types
#define LEFT_ARM 0
#define RIGHT_ARM 1

//The joint labels:
//LEFT
#define SHOULDER_GOLD 0
#define ELBOW_GOLD 1
#define Z_INS_GOLD 2
#define TOOL_ROT_GOLD 4
#define WRIST_GOLD 5
#define GRASP1_GOLD 6
#define GRASP2_GOLD 7
//RIGHT
#define SHOULDER_GREEN 8
#define ELBOW_GREEN 9
#define Z_INS_GREEN 10
#define TOOL_ROT_GREEN 12
#define WRIST_GREEN 13
#define GRASP1_GREEN 14
#define GRASP2_GREEN 15

//ROS publish 
#define ROS_PUBLISH_RATE 1000 	// in Hz

using namespace raven_2;
using namespace std;

//State machine modes
enum mode {
	idle,
	joint_control,
	snake_teleop
};

class Raven_Controller
{
	private:
		int PUB_COUNT;
		int SUB_COUNT;
		bool SHOW_STATUS;
		bool RECEIVED_FIRST;
		bool PAUSE;

		pthread_t console_thread;
		pthread_t ros_thread;

		ros::Publisher RavenAutomove_publisher;
		ros::Subscriber RavenState_subscriber;

		raven_state CURR_RAVEN_STATE;

		//Snake Control
		SnakeRaven GoldSnake;
		SnakeRaven GreenSnake;
		Matrix4d Ttarget;
		MatrixXd dx;
		double dx_max;
		mode state;

	public:
		Raven_Controller();		// constructor
		void initial(int, char**);	// initialization and console display
		void init_sys();
		bool init_ros(int, char**);
		void init_words();
		bool menu_words(bool);
		bool joint_menu_words(bool);
		bool teleop_menu_words(bool);
		void final_words();

		//State machine functions
		void jointcontrol_keyboard_map(int);
		void updateSnakejoints();
		void keyboard_teleop_map(int);
		void updateRavenState();

		void start_thread();		// thread management
		void join_thread();
		void *console_process(void);
		void *ros_process(void);
		static void *static_console_process(void*);
		static void *static_ros_process(void*);

		void publish_raven_automove();			 // ROS publish
		void callback_raven_state(raven_2::raven_state); // ROS subscribe

		void output_PUBinfo();
		void output_SUBinfo();

		int getKey();

}; //end of class definition

#endif
