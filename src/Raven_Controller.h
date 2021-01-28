#ifndef RAVEN_CONTROLLER_H_
#define RAVEN_CONTROLLER_H_

//Raven dependencies
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

//ROS messages
#include <sensor_msgs/JointState.h>
#include "snake_raven_controller/raven_jointmove.h"
#include "snake_raven_controller/snakeraven_state.h"

//Snake Raven Class
#include "SnakeRaven.h" // Snake Raven Class
#include <cmath> //rounding

//Debugging mode
#include <unistd.h> //usleep for debugging
#include <inttypes.h> //for printing time to csv
//#define Debugging //goes through the debugging process if its defined

//Debugging snippet
//#ifdef Debugging
//cout<< "Start loop: Reading mv_pre..."<<endl;
//cout<<GoldSnake.mv_pre<<endl;
//cout<<GreenSnake.mv_pre<<endl;
//usleep(2000000);
//#endif

//Arm types
#define LEFT_ARM 0
#define RIGHT_ARM 1

//The joint labels:
//LEFT
#define SHOULDER_GOLD 0
#define ELBOW_GOLD 1
#define Z_INS_GOLD 2
#define TOOL_ROT_GOLD 3
#define WRIST_GOLD 4
#define GRASP1_GOLD 5
#define GRASP2_GOLD 6
//RIGHT
#define SHOULDER_GREEN 7
#define ELBOW_GREEN 8
#define Z_INS_GREEN 9
#define TOOL_ROT_GREEN 10
#define WRIST_GREEN 11
#define GRASP1_GREEN 12
#define GRASP2_GREEN 13

//ROS publish //1000 on Raven computer, 100 on laptop
#define ROS_PUBLISH_RATE 1000 	// in Hz

//using namespace raven_2;
using namespace std;
using namespace snake_raven_controller;

//State machine modes
enum mode {
	idle,
	calibration,
	joint_control,
	calibrated_motion,
	snake_teleop
};

class Raven_Controller
{
	private:
		int PUB_COUNT;
		int SUB_COUNT;
		int checkpoint;
		int counter;
		
		bool SHOW_STATUS;
		bool RECEIVED_FIRST;
		bool CALIBRATED;
		bool NO_TARGET_SET;
		bool GO_HOME;
		bool MANUAL;

		pthread_t console_thread;
		pthread_t ros_thread;

		ros::Publisher RavenJointmove_publisher;
		ros::Subscriber RavenJoint_subscriber;

		//Record data publisher
		ros::Publisher Snakeraven_publisher;

		//Record data into file:
		FILE * pFile;
		bool RECORDING;
		float record_freq;

		//Length of Snake Tool
		float snake_length;

		//Variable to publish joint deltas
		float delta_joint[14];

		//Calibration
		double mv2jpos_rate[14];
		double calibrate_offset[14];

		//Joint State subscriber values
		double jpos_initial[14];
		double jpos_current[14];
		double jpos_desired[14];

		//Controller settings:
		double dx_max; //mm
		double tol_error; //magnitude error
		double motor_maxspeed_rad1; //rad for larger joints
		double motor_maxspeed_rad2; //rad for smaller joints
		double motor_maxspeed_mm; //mm
		float factor;

		//Debugging tool
		string text;

		//Snake Control
		SnakeRaven GoldSnake;
		SnakeRaven GreenSnake;
		Matrix4d Ttarget;
		Matrix4d Tend;
		MatrixXd dx;
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
		bool calibration_menu_words(bool); 
		bool manual_calibration_words(bool);
		bool move_2_home();
		void final_words();

		//State machine functions
		void jointcontrol_keyboard_map(int);
		void updateSnakejoints();

		MatrixXd keyboard_teleop_map(int, const MatrixXd&);
		double saturate_round(double, float);
		void updateRavenJointarray();
		void calibrate_snake_raven_state();

		void start_thread();		// thread management
		void join_thread();
		void *console_process(void);
		void *ros_process(void);
		static void *static_console_process(void*);
		static void *static_ros_process(void*);

		void publish_raven_jointmove();			 // ROS publish
		void callback_joint_states(sensor_msgs::JointState); //ROS subscribe joint states

		//Record data
		void publish_snakeraven_state();
		void record_to_csv();

		void output_PUBinfo();
		void output_SUBinfo();

		int getKey();

}; //end of class definition

#endif
