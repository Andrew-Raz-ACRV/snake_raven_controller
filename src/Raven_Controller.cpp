#include "Raven_Controller.h"

/**
*	\fn Raven_Controller()
*
* 	\brief this is the constructor
*
* 	\param void
*
*	\return none
*/
Raven_Controller::Raven_Controller()
{

}




/**
*	\fn initial()
*
* 	\brief initialize everything for this program.
*
* 	\param int argc, char** argv
*
*	\return void
*/
void Raven_Controller::initial(int argc, char** argv)
{
	init_sys();
	if(!init_ros(argc,argv))
  	{
     		ROS_ERROR("Fail to initialize ROS. Exiting!");
		exit(1);
  	}
	
	//print the welcome greets on console
	init_words();
}



/**
*	\fn void init_sys() 
*
*	\brief initialize default system parameter settings.
*
* 	\param void
*
*	\return void
*/

void Raven_Controller::init_sys()  
{
	this->PUB_COUNT = 0;
	this->SUB_COUNT = 0;
	this->KEY = 0;

	this->RECEIVED_FIRST = false;
	this->SHOW_STATUS = false;
	this->PAUSE = false;

	this->CALIBRATED = false;

	//Initialise the joint delta array
	for(int i = 0; i < 16; i++){
		this->delta_joint[i] = 0;
		this->q_initial[i] = 0;
	}

	//Home Configuration for calibration
	this->q_initial[SHOULDER_GOLD] = deg2rad(39.5967);
	this->q_initial[ELBOW_GOLD] = deg2rad(-102.0840);
	this->q_initial[SHOULDER_GREEN] = deg2rad(-39.5967);
	this->q_initial[ELBOW_GREEN] = deg2rad(-77.9160);
}




/**
*	\fn int init_ros(int argc, char** argv) 
*
*	\brief initialize ROS and connect the pub & sub relation.
*
* 	\param int argc, char** argv
*
*	\return bool
*/
bool Raven_Controller::init_ros(int argc, char** argv) 
{
	//initialize ROS
	ros::init(argc, argv, "snake_raven_controller"); //"AutoCircle_generator"

	static ros::NodeHandle n;
  	RavenJointmove_publisher = n.advertise<raven_jointmove>("raven_jointmove", 1);
	RavenState_subscriber   = n.subscribe("ravenstate",1,&Raven_Controller::callback_raven_state,this);

	return true;
}



/**
*	\fn void init_words() 
*
*	\brief show greeting words on console.
*
* 	\param void
*
*	\return void
*/
void Raven_Controller::init_words()  
{
	string start = "0";
	do
	{
		cout<<endl<<endl;
		cout<<"Welcome to the Snake Raven Controller"<<endl<<endl;
		cout<<"Please press \"Enter\" to start!";
		cin.clear();
		getline(std::cin,start);
	}while(start!="");

	cout<<"SnakeRaven Talker should be starting..."<<endl;
}



/**
*	\fn void menu_words() 
*
*	\brief show menu words on console.
*
* 	\param void
*
*	\return void
*/
bool Raven_Controller::menu_words(bool print_menu)  
{
	if(print_menu)
	{
		cout<<endl;
		cout<<"Snake Raven Controller Main Menu:"<<endl;
		cout<<"----------------------------------------------------"<<endl;
		cout<<"\t'0' : Set up Calibration"<<endl;
		cout<<"\t'1' : Joint Controller mode"<<endl;
		cout<<"\t'2' : Keyboard Teleoperation mode"<<endl;
		cout<<"\t'^C': Quit program"<<endl<<endl;
	}
	return false;
}

/**
*	\fn void menu_words() 
*
*	\brief show menu words on console.
*
* 	\param void
*
*	\return void
*/
bool Raven_Controller::calibration_menu_words(bool print_menu)  
{
	if(print_menu)
	{
		cout<<endl;
		cout<<"To calibrate control the RAVEN 2 joints to make the"<<endl;
		cout<<"snake tool completely vertical facing down...      "<<endl;
		cout<<"The z_insertion should place the base of the snake "<<endl;
		cout<<"at the RCM point                                          "<<endl;
		cout<<" - press 4 or f to check positive pan rotation            "<<endl;
		cout<<"   x axis is coming out front of raven (back for left)    "<<endl;
		cout<<" - press 5 or g to check positive tilt rotation           "<<endl;
		cout<<"   y axis is coming out left of raven (right for left arm)"<<endl;
		cout<<" - do similar next joints, its rotated by 45 degrees in up z axis"<<endl;
		cout<<"                                                      "<<endl;
		cout<<"                          press 'k' when finished     "<<endl;
		cout<<"-----------------------------------------------------------------"<<endl;
		cout<<"\t'1 2 3 4 5 6 7' : Increase Gold arm DOF"<<endl;
		cout<<"\t'q w e r t y u' : Decrease Gold arm DOF"<<endl;
		cout<<"\t'a s d f g h j' : Increase Green arm DOF"<<endl;
		cout<<"\t'z x c v b n m' : Decrease Green arm DOF"<<endl;
		cout<<"\t'k' : finish and return to main menu"<<endl;
		cout<<"\t'^C': Quit"<<endl<<endl;
	}
	return false;
}

/**
*	\fn void menu_words() 
*
*	\brief show menu words on console.
*
* 	\param void
*
*	\return void
*/
bool Raven_Controller::joint_menu_words(bool print_menu)  
{
	if(print_menu)
	{
		cout<<endl;
		cout<<"Snake Raven Joint controller Selection Menu:"<<endl;
		cout<<"----------------------------------------------------"<<endl;
		cout<<"\t'1 2 3 4 5 6 7' : Increase Gold arm DOF"<<endl;
		cout<<"\t'q w e r t y u' : Decrease Gold arm DOF"<<endl;
		cout<<"\t'a s d f g h j' : Increase Green arm DOF"<<endl;
		cout<<"\t'z x c v b n m' : Decrease Green arm DOF"<<endl;
		cout<<"\t'k' : return to main menu"<<endl;
		cout<<"\t'^C': Quit"<<endl<<endl;
	}
	return false;
}

/**
*	\fn void menu_words() 
*
*	\brief show menu words on console.
*
* 	\param void
*
*	\return void
*/
bool Raven_Controller::teleop_menu_words(bool print_menu)  
{
	if(print_menu)
	{
		cout<<endl;
		cout<<"Snake Raven Teleoperation Selection Menu:"<<endl;
		cout<<"Moves Green Arm only:"<<endl;
		cout<<"----------------------------------------------------"<<endl;
		cout<<"\t'a d' : -/+ y axis"<<endl;
		cout<<"\t'w s' : +/- x axis"<<endl;
		cout<<"\t'q e' : +/- z axis"<<endl;
		cout<<"\t'f h' : +/- y rotation"<<endl;
		cout<<"\t'g t' : +/- x rotation"<<endl;
		cout<<"\t'r y' : +/- z rotation"<<endl;
		cout<<"\t'z' : reset target pose"<<endl;		
		cout<<"\t'k' : return to main menu"<<endl;
		cout<<"\t'^C': Quit"<<endl<<endl;


		//Initialise teleop

		//INITIALISE THE TARGET GREEN ARM ONLY
		//updateSnakejoints();
		//Ttarget = GreenSnake.FK();
	}
	return false;
}



/**
*	\fn void final_words() 
*
*	\brief show goodbye words on console.
*
* 	\param void
*
*	\return void
*/
void Raven_Controller::final_words()  
{
	cout<<"Terminating the Snake Raven Controller." <<endl;
	cout<<"----------------------------------------------------"<<endl;
	cout<<"GoodBye!"<<endl<<endl<<endl;
}




/**
*	\fn void start_thread()
*
* 	\brief start the console_thread and ros_thread
*
* 	\param void
*
*	\return void
*/
void Raven_Controller::start_thread()
{
	pthread_create(&console_thread,NULL,Raven_Controller::static_console_process,this);
	pthread_create(&ros_thread,NULL,Raven_Controller::static_ros_process,this);

}



/**
*	\fn void join_thread()
*
* 	\brief join the console_thread and ros_thread
*
* 	\param void
*
*	\return void
*/
void Raven_Controller::join_thread()
{
	pthread_join(console_thread,NULL);
	pthread_join(ros_thread,NULL);

	final_words();
}



/**
*	\fn void *console_process(void)
*
* 	\brief this thread is dedicated to console io
*
* 	\param a pointer to void
*
*	\return void
*/
void* Raven_Controller::console_process(void)
{
	if(ros::isInitialized())
	{
		int theKey;
		bool print_menu = true;
		ros::Time time;
		time = time.now();
		state = idle;

		while(ros::ok())
		{
			theKey = getKey();	

			if(!RECEIVED_FIRST){
				cout<<"Waiting for the first receive of raven_state..."<<endl;
			}
				
			//State machine	
			switch(state)
			{
				case idle:
				{
					print_menu = menu_words(print_menu);
					//check theKey to see if the state changes
					if(theKey=='0')
					{
						//change state to joint control
						state = calibration;
						print_menu = true;
					}
					if(theKey=='1')
					{
						//change state to joint control
						state = joint_control;
						print_menu = true;
					}
					else if(theKey=='2')
					{
						if(CALIBRATED&&RECEIVED_FIRST){
							//change state to teleop control
							state = snake_teleop;	
							print_menu = true;
						}
						else{
							cout<<"\nCALIBRATION required before teleoperation"<<endl;
						}
					}

					break;
				}
				case calibration:
				{
					//Explain calibration procedure
					print_menu = calibration_menu_words(print_menu);

					//Joint control state
					jointcontrol_keyboard_map(theKey);

					//kill state
					if(theKey=='k')
					{
						state = idle;
						print_menu = true;
						//SET UP CONVERSION from jpos to q
						calibrate_snake_raven_state();
						CALIBRATED = true;
						cout<<"\nCalibration Set!"<<endl;
						cout<<"\nExiting mode..."<<endl;
					}
					break;
				}
				case joint_control:
				{
					//one time menu for joint control
					print_menu = joint_menu_words(print_menu);

					//Joint control state
					jointcontrol_keyboard_map(theKey);

					//kill state
					if(theKey=='k')
					{
						state = idle;
						print_menu = true;
						cout<<"\nExiting mode..."<<endl;
					}
					break;
				}
				case snake_teleop:
				{
					//one time menu for teleoperation
					print_menu = teleop_menu_words(print_menu);

					//Read current state:
					updateSnakejoints();

					//Update target and compute joint increment
					GreenSnake.Tend = GreenSnake.FK(); //Update Forward Kinematics
					GoldSnake.Tend = GoldSnake.FK(); //Update Forward Kinematics

					Ttarget = keyboard_teleop_map(theKey,GreenSnake.Tend);
					dx = trans2dx(GreenSnake.Tend, Ttarget); //Update current Error
					cout << "Error = " << dx.norm() << endl;
					if (dx.norm() < tol_error) {
						cout << "Target Region Reached" << endl;
						cout << Ttarget << endl;
					}
					else {
						if (dx.norm() > dx_max) {
							dx = cap_mag(dx, dx_max); //Restrict delta size
						}
						GreenSnake.J = GreenSnake.Jacobian(); //Update Jacobian
						GreenSnake.q += dampedLeastSquares(GreenSnake.J, GreenSnake.q, GreenSnake.ql, GreenSnake.qu) * dx; //Joint Delta update
						GreenSnake.q = applyJointLimits(GreenSnake.q, GreenSnake.ql, GreenSnake.qu); //Joint limits

						//Compute Motor values
						GreenSnake.mv = GreenSnake.q2Motor_angles(); //Compute Motor values
						GoldSnake.mv = GoldSnake.q2Motor_angles(); //Compute Motor values

						updateRavenJointarray(); // send computed motor values into raven state
					}

					//kill state
					if(theKey=='k')
					{
						state = idle;
						print_menu = true;
						cout<<"\nExiting mode..."<<endl;
					}
					break;
				}
			}

		}
		cout<<"console_process is shutdown."<<endl;

	}
	return 0;
}



/**
*	\fn void *ros_process(void)
*
* 	\brief this thread is dedicated to ros pub & sub
*
* 	\param a pointer to void
*
*	\return void
*/
void* Raven_Controller::ros_process(void)
{
	if(ros::isInitialized())
	{
		if(!RECEIVED_FIRST){
			cout<<endl<<"Waiting for the first receive of raven_state..."<<endl;
		}
		else
		{
			cout<<"First raven_state received."<<endl;
		}

		while(ros::ok() && RECEIVED_FIRST)
		{
			//cout<<"Publishing raven_automove... "<<endl;
			publish_raven_jointmove();
			//output_PUBinfo();
		}

		if(RECEIVED_FIRST)
			cout<<"ros_process is shutdown."<<endl;
	}
	return 0;
}

/**
*	\fn keyboard_teleop_map
*
* 	\updates the target position based on keypresses
*
* 	\param int theKey from console, matrix T.
*
*	\return Ttarget Matrix4d
*/
MatrixXd Raven_Controller::keyboard_teleop_map(int theKey, const MatrixXd& T)
{
	//Keyboard update step
	double delta = 1;
	Matrix4d Ttarget = T;

	switch(theKey)
	{
		//Teleoperation mapping
		case 'a':
		{
			//Y--
			Ttarget(1, 3) -= delta;
			//cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case 'd':
		{
			//Y++
			Ttarget(1, 3) += delta;
			//cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case 'w':
		{
			//X++
			Ttarget(0, 3) += delta;
			//cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case 's':
		{
			//X--
			Ttarget(0, 3) -= delta;
			//cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case 'q':
		{
			//Z++
			Ttarget(2, 3) += delta;
			//cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case 'e':
		{
			//Z--
			Ttarget(2, 3) -= delta;
			//cout << "Target= " << endl << Ttarget << endl;
			break;
		}
	}
	return Ttarget;
}



void * Raven_Controller::static_console_process(void* classRef)
{
	return ((Raven_Controller *)classRef)->console_process();
}



void * Raven_Controller::static_ros_process(void* classRef)
{
	return ((Raven_Controller *)classRef)->ros_process();
}



/**
*	\fn void publish_raven_automove()
*
*	\brief publish the auto circle command to raven_automove topic on ROS.
*
* 	\param int
*
*	\return void
*/
void Raven_Controller::publish_raven_jointmove()
{
	static ros::Rate loop_rate(ROS_PUBLISH_RATE);
	static raven_jointmove msg_raven_jointmove;	

	// (1) wrap up the new command	
	//msg_raven_jointmove.hdr.stamp = msg_raven_jointmove.hdr.stamp.now(); //hdr

	//JOINT POSITION PUBLISH
	//Left arm
	msg_raven_jointmove.delta_joint[SHOULDER_GOLD] = delta_joint[SHOULDER_GOLD];
	msg_raven_jointmove.delta_joint[ELBOW_GOLD] = delta_joint[ELBOW_GOLD];
	msg_raven_jointmove.delta_joint[Z_INS_GOLD] = delta_joint[Z_INS_GOLD];
	msg_raven_jointmove.delta_joint[TOOL_ROT_GOLD] = delta_joint[TOOL_ROT_GOLD];
	msg_raven_jointmove.delta_joint[WRIST_GOLD] = delta_joint[WRIST_GOLD];
	msg_raven_jointmove.delta_joint[GRASP1_GOLD] = delta_joint[GRASP1_GOLD];
	msg_raven_jointmove.delta_joint[GRASP2_GOLD] = delta_joint[GRASP2_GOLD];

	//Right arm
	msg_raven_jointmove.delta_joint[SHOULDER_GREEN] = delta_joint[SHOULDER_GREEN];
	msg_raven_jointmove.delta_joint[ELBOW_GREEN] = delta_joint[ELBOW_GREEN];
	msg_raven_jointmove.delta_joint[Z_INS_GREEN] = delta_joint[Z_INS_GREEN];
	msg_raven_jointmove.delta_joint[TOOL_ROT_GREEN] = delta_joint[TOOL_ROT_GREEN];
	msg_raven_jointmove.delta_joint[WRIST_GREEN] = delta_joint[WRIST_GREEN];
	msg_raven_jointmove.delta_joint[GRASP1_GREEN] = delta_joint[GRASP1_GREEN];
	msg_raven_jointmove.delta_joint[GRASP2_GREEN] = delta_joint[GRASP2_GREEN];

	//Reset delta_array to 0 for next loop
	for (int i = 0; i<16; i++){
		delta_joint[i] = 0;
	}

	// (2) send new command
	RavenJointmove_publisher.publish(msg_raven_jointmove);
	ros::spinOnce();

	//(3) prepare for next publish
	loop_rate.sleep();
	PUB_COUNT ++;
}



/**
*	\fn void autoRavenStateCallback(raven_2::raven_state msg)
*
*	\brief This function is automatically called whenever someone publish to raven_state topic
*
* 	\param raven_2::raven_state msg
*
*	\return void
*/
void Raven_Controller::callback_raven_state(raven_2::raven_state msg) 
{
	// (1) save the updated raven_state 
	CURR_RAVEN_STATE.runlevel = msg.runlevel;
	CURR_RAVEN_STATE.sublevel = msg.sublevel;
	CURR_RAVEN_STATE.last_seq = msg.last_seq;
	CURR_RAVEN_STATE.dt = msg.dt;

	//Save joint update Raven State jpos() THE CURRENT JOINT ANGLE
	//Left arm
	CURR_RAVEN_STATE.jpos[SHOULDER_GOLD] = msg.jpos[SHOULDER_GOLD];
	CURR_RAVEN_STATE.jpos[ELBOW_GOLD] = msg.jpos[ELBOW_GOLD];
	CURR_RAVEN_STATE.jpos[Z_INS_GOLD] = msg.jpos[Z_INS_GOLD];
	CURR_RAVEN_STATE.jpos[TOOL_ROT_GOLD] = msg.jpos[TOOL_ROT_GOLD];
	CURR_RAVEN_STATE.jpos[WRIST_GOLD] = msg.jpos[WRIST_GOLD];
	CURR_RAVEN_STATE.jpos[GRASP1_GOLD] = msg.jpos[GRASP1_GOLD];
	CURR_RAVEN_STATE.jpos[GRASP2_GOLD] = msg.jpos[GRASP2_GOLD];
	//Right arm
	CURR_RAVEN_STATE.jpos[SHOULDER_GREEN] = msg.jpos[SHOULDER_GREEN];
	CURR_RAVEN_STATE.jpos[ELBOW_GREEN] = msg.jpos[ELBOW_GREEN];
	CURR_RAVEN_STATE.jpos[Z_INS_GREEN] = msg.jpos[Z_INS_GREEN];
	CURR_RAVEN_STATE.jpos[TOOL_ROT_GREEN] = msg.jpos[TOOL_ROT_GREEN];
	CURR_RAVEN_STATE.jpos[WRIST_GREEN] = msg.jpos[WRIST_GREEN];
	CURR_RAVEN_STATE.jpos[GRASP1_GREEN] = msg.jpos[GRASP1_GREEN];
	CURR_RAVEN_STATE.jpos[GRASP2_GREEN] = msg.jpos[GRASP2_GREEN];

	// (2) update recieved data count
	SUB_COUNT ++;
	RECEIVED_FIRST = true;
}

/*
/ Brief: determines the offsets from home position set in the calibration mode
/
/
/
*/
void Raven_Controller::calibrate_snake_raven_state()
{
	//We know the current raven_state is equal to the initial q 
	//when this function is called in calibration
	//
	// q = calibrate_rate * CURR_RAVEN_STATE + calibrate_offset;
	//
	// CURR_RAVEN_STATE = (q - offset)/rate

	//since they are in radians, rate is 1 except insertion converts from m to mm
	for (int i = 0; i<16; i++){
		calibrate_rate[i] = 1;
	}
	calibrate_rate[Z_INS_GOLD] = 1000;
	calibrate_rate[Z_INS_GREEN] = 1000;

	//Compute offset
	// offset = q - rate*CURR
	for (int i = 0; i<16; i++){
		calibrate_offset[i] = q_initial[i] - calibrate_rate[i]*CURR_RAVEN_STATE.jpos[i];
	}

	/*

	//Print 
	cout<<"calibrate_offset = "<<endl;
	for (int i = 0; i<16; i++){
		cout<<calibrate_offset[i]<<endl;
	}

	cout<<"q = "<<GreenSnake.q<<endl<<endl;

	cout<<"CURR_RAVEN_STATE = "<<endl;
	for (int i = 0; i<16; i++){
		cout<<CURR_RAVEN_STATE.jpos[i]<<endl;
	}

	cout<<"NEXT_RAVEN_STATE green = "<<endl;
	for (int i = 8; i<13; i++){
		cout<<(GreenSnake.mv(i - 8) - calibrate_offset[i])/calibrate_rate[i]<<endl;
	}

	cout<<"delta_ green = "<<endl;
	for (int i = 8; i<13; i++){
		cout<<saturate_round((GreenSnake.mv(i - 8) - calibrate_offset[i])/calibrate_rate[i] - CURR_RAVEN_STATE.jpos[i])<<endl;
	}
	*/

	//

}

/**
*	\fn void output_PUBinfo()
*
* 	\brief shows the publish status (display every one sec)
*
* 	\param void
*
*	\return void
*/
void Raven_Controller::output_PUBinfo()
{
	ROS_INFO("snake_raven_controller publish: raven_jointmove[%d]", PUB_COUNT);
}



/**
*	\fn void output_SUBinfo()
*
* 	\brief shows the subscribe status (display every one sec)
*
* 	\param void
*
*	\return void
*/
void Raven_Controller::output_SUBinfo()
{
	ROS_INFO("snake_raven_controller subscribe: raven_state[%d]", SUB_COUNT);

	//Raven joint positions
		//Joint position update:
	for (int i=0; i<2; i++){
		//per arm
		if(i==0){
			cout<<"\n\tLEFT ARM data1.jpos_d = "<<endl;	
			for (int j=0; j<8; j++){
				//Per DOF
				cout<<"\tDOF "<<j<<" = "<<CURR_RAVEN_STATE.jpos_d[i*SHOULDER_GREEN + j]<<endl;
			}
		}
		else if(i==1){
			cout<<"\n\tRIGHT ARM data1.jpos_d = "<<endl;
			for (int j=0; j<8; j++){
				//Per DOF
				cout<<"\tDOF "<<j<<" = "<<CURR_RAVEN_STATE.jpos_d[i*SHOULDER_GREEN + j]<<endl;
			}
		}
	}
	cout<<endl;
	cout<<endl;
}



/**
*	\fn int getKey()
*
*	\brief gets keyboard character for switch case's of console_process()
*
* 	\param void
*
*	\return character int
*/
int Raven_Controller::getKey() 
{
    	int character;
    	struct termios orig_term_attr;
    	struct termios new_term_attr;

    	// set the terminal to raw mode 
    	tcgetattr(fileno(stdin), &orig_term_attr);
    	memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    	new_term_attr.c_lflag &= ~(ECHO|ICANON);
    	new_term_attr.c_cc[VTIME] = 0;
    	new_term_attr.c_cc[VMIN] = 0;
    	tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    	// read a character from the stdin stream without blocking 
    	//   returns EOF (-1) if no character is available 
    	character = fgetc(stdin);

   	// restore the original terminal attributes 
    	tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    	return character;
}

//****************Snake Raven new functions****************//

/*
* \brief: puts the raven_state data into the two snakeraven class objects
* 
* \param void
*
* \return void
*/
void Raven_Controller::updateSnakejoints()
{
	//Use Calibration q = rate*jpos + offset

	//Put CURR_RAVEN_STATE into Snake Raven joint vector objects
	//Only the first 3 DOF because there is no reverse reading 
	GoldSnake.q(0) = calibrate_rate[SHOULDER_GOLD]*CURR_RAVEN_STATE.jpos[SHOULDER_GOLD] + calibrate_offset[SHOULDER_GOLD];
	GoldSnake.q(1) = calibrate_rate[ELBOW_GOLD]*CURR_RAVEN_STATE.jpos[ELBOW_GOLD] + calibrate_offset[ELBOW_GOLD];
	GoldSnake.q(2) = calibrate_rate[Z_INS_GOLD]*CURR_RAVEN_STATE.jpos[Z_INS_GOLD] + calibrate_offset[Z_INS_GOLD];

	GreenSnake.q(0) = calibrate_rate[SHOULDER_GREEN]*CURR_RAVEN_STATE.jpos[SHOULDER_GREEN] + calibrate_offset[SHOULDER_GREEN];
	GreenSnake.q(1) = calibrate_rate[ELBOW_GREEN]*CURR_RAVEN_STATE.jpos[ELBOW_GREEN] + calibrate_offset[ELBOW_GREEN];
	GreenSnake.q(2) = calibrate_rate[Z_INS_GREEN]*CURR_RAVEN_STATE.jpos[Z_INS_GREEN] + calibrate_offset[Z_INS_GREEN];

	//Note that there is no reverse mapping from Raven motor value to pan and tilt q angles
	//assume initially zeros from snakeRaven constructor or that the last q(3-6) is correct
}

float Raven_Controller::saturate_round(float input, float limit)
{
	float output;
	//round the input
	float value = (int)(input * 10000 + .5); 

	output = (float)value / 10000;

	//saturate
	if (output>limit){
		output = limit;
	}
	else if (output<-limit){
		output = -limit;
	}

    return output;
}

void Raven_Controller::updateRavenJointarray()
{
	//turn joint vector into a joint delta update, depends on how many modules of the snake tool

	//Idea:
	//Next_joint = (q - offset)/rate;
	//delta_joint = Next_joint - CURR_joint;

	//Left arm
	if (GoldSnake.m==2){
		//SHOULDER_GOLD
		NEXT_JOINT_STATE[SHOULDER_GOLD]= (GoldSnake.mv(0) - calibrate_offset[SHOULDER_GOLD])/calibrate_rate[SHOULDER_GOLD];
		delta_joint[SHOULDER_GOLD] = saturate_round(NEXT_JOINT_STATE[SHOULDER_GOLD] - CURR_RAVEN_STATE.jpos[SHOULDER_GOLD], motor_maxspeed_rad);
		//ELBOW_GOLD
		NEXT_JOINT_STATE[ELBOW_GOLD]= (GoldSnake.mv(1) - calibrate_offset[ELBOW_GOLD])/calibrate_rate[ELBOW_GOLD];
		delta_joint[ELBOW_GOLD] = saturate_round(NEXT_JOINT_STATE[ELBOW_GOLD] - CURR_RAVEN_STATE.jpos[ELBOW_GOLD], motor_maxspeed_rad);
		//Z_INS_GOLD
		NEXT_JOINT_STATE[Z_INS_GOLD]= (GoldSnake.mv(2) - calibrate_offset[Z_INS_GOLD])/calibrate_rate[Z_INS_GOLD];
		delta_joint[Z_INS_GOLD] = saturate_round(NEXT_JOINT_STATE[Z_INS_GOLD] - CURR_RAVEN_STATE.jpos[Z_INS_GOLD], motor_maxspeed_mm);
		//TOOL_ROT_GOLD
		NEXT_JOINT_STATE[TOOL_ROT_GOLD]= (GoldSnake.mv(3) - calibrate_offset[TOOL_ROT_GOLD])/calibrate_rate[TOOL_ROT_GOLD];
		delta_joint[TOOL_ROT_GOLD] = saturate_round(NEXT_JOINT_STATE[TOOL_ROT_GOLD] - CURR_RAVEN_STATE.jpos[TOOL_ROT_GOLD], motor_maxspeed_rad);
		//WRIST_GOLD
		NEXT_JOINT_STATE[WRIST_GOLD]= (GoldSnake.mv(4) - calibrate_offset[WRIST_GOLD])/calibrate_rate[WRIST_GOLD];
		delta_joint[WRIST_GOLD] = saturate_round(NEXT_JOINT_STATE[WRIST_GOLD] - CURR_RAVEN_STATE.jpos[WRIST_GOLD], motor_maxspeed_rad);
		//GRASP1_GOLD
		NEXT_JOINT_STATE[GRASP1_GOLD]= (GoldSnake.mv(5) - calibrate_offset[GRASP1_GOLD])/calibrate_rate[GRASP1_GOLD];
		delta_joint[GRASP1_GOLD] = saturate_round(NEXT_JOINT_STATE[GRASP1_GOLD] - CURR_RAVEN_STATE.jpos[GRASP1_GOLD], motor_maxspeed_rad);
		//GRASP2_GOLD
		NEXT_JOINT_STATE[GRASP2_GOLD]= (GoldSnake.mv(6) - calibrate_offset[GRASP2_GOLD])/calibrate_rate[GRASP2_GOLD];
		delta_joint[GRASP2_GOLD] = saturate_round(NEXT_JOINT_STATE[GRASP2_GOLD] - CURR_RAVEN_STATE.jpos[GRASP2_GOLD], motor_maxspeed_rad);

	}
	else if (GoldSnake.m==1){
		//SHOULDER_GOLD
		NEXT_JOINT_STATE[SHOULDER_GOLD]= (GoldSnake.mv(0) - calibrate_offset[SHOULDER_GOLD])/calibrate_rate[SHOULDER_GOLD];
		delta_joint[SHOULDER_GOLD] = saturate_round(NEXT_JOINT_STATE[SHOULDER_GOLD] - CURR_RAVEN_STATE.jpos[SHOULDER_GOLD], motor_maxspeed_rad);
		//ELBOW_GOLD
		NEXT_JOINT_STATE[ELBOW_GOLD]= (GoldSnake.mv(1) - calibrate_offset[ELBOW_GOLD])/calibrate_rate[ELBOW_GOLD];
		delta_joint[ELBOW_GOLD] = saturate_round(NEXT_JOINT_STATE[ELBOW_GOLD] - CURR_RAVEN_STATE.jpos[ELBOW_GOLD], motor_maxspeed_rad);
		//Z_INS_GOLD
		NEXT_JOINT_STATE[Z_INS_GOLD]= (GoldSnake.mv(2) - calibrate_offset[Z_INS_GOLD])/calibrate_rate[Z_INS_GOLD];
		delta_joint[Z_INS_GOLD] = saturate_round(NEXT_JOINT_STATE[Z_INS_GOLD] - CURR_RAVEN_STATE.jpos[Z_INS_GOLD], motor_maxspeed_mm);
		//TOOL_ROT_GOLD
		NEXT_JOINT_STATE[TOOL_ROT_GOLD]= (GoldSnake.mv(3) - calibrate_offset[TOOL_ROT_GOLD])/calibrate_rate[TOOL_ROT_GOLD];
		delta_joint[TOOL_ROT_GOLD] = saturate_round(NEXT_JOINT_STATE[TOOL_ROT_GOLD] - CURR_RAVEN_STATE.jpos[TOOL_ROT_GOLD], motor_maxspeed_rad);
		//WRIST_GOLD
		NEXT_JOINT_STATE[WRIST_GOLD]= (GoldSnake.mv(4) - calibrate_offset[WRIST_GOLD])/calibrate_rate[WRIST_GOLD];
		delta_joint[WRIST_GOLD] = saturate_round(NEXT_JOINT_STATE[WRIST_GOLD] - CURR_RAVEN_STATE.jpos[WRIST_GOLD], motor_maxspeed_rad);
		//GRASP1_GOLD
		delta_joint[GRASP1_GOLD] = 0;
		//GRASP2_GOLD
		delta_joint[GRASP2_GOLD] = 0;

	}

	//Right arm
	if (GreenSnake.m==2){
		//SHOULDER_GREEN
		NEXT_JOINT_STATE[SHOULDER_GREEN]= (GreenSnake.mv(0) - calibrate_offset[SHOULDER_GREEN])/calibrate_rate[SHOULDER_GREEN];
		delta_joint[SHOULDER_GREEN] = saturate_round(NEXT_JOINT_STATE[SHOULDER_GREEN] - CURR_RAVEN_STATE.jpos[SHOULDER_GREEN], motor_maxspeed_rad);
		//ELBOW_GREEN
		NEXT_JOINT_STATE[ELBOW_GREEN]= (GreenSnake.mv(1) - calibrate_offset[ELBOW_GREEN])/calibrate_rate[ELBOW_GREEN];
		delta_joint[ELBOW_GREEN] = saturate_round(NEXT_JOINT_STATE[ELBOW_GREEN] - CURR_RAVEN_STATE.jpos[ELBOW_GREEN], motor_maxspeed_rad);
		//Z_INS_GREEN
		NEXT_JOINT_STATE[Z_INS_GREEN]= (GreenSnake.mv(2) - calibrate_offset[Z_INS_GREEN])/calibrate_rate[Z_INS_GREEN];
		delta_joint[Z_INS_GREEN] = saturate_round(NEXT_JOINT_STATE[Z_INS_GREEN] - CURR_RAVEN_STATE.jpos[Z_INS_GREEN], motor_maxspeed_mm);
		//TOOL_ROT_GREEN
		NEXT_JOINT_STATE[TOOL_ROT_GREEN]= (GreenSnake.mv(3) - calibrate_offset[TOOL_ROT_GREEN])/calibrate_rate[TOOL_ROT_GREEN];
		delta_joint[TOOL_ROT_GREEN] = saturate_round(NEXT_JOINT_STATE[TOOL_ROT_GREEN] - CURR_RAVEN_STATE.jpos[TOOL_ROT_GREEN], motor_maxspeed_rad);
		//WRIST_GREEN
		NEXT_JOINT_STATE[WRIST_GREEN]= (GreenSnake.mv(4) - calibrate_offset[WRIST_GREEN])/calibrate_rate[WRIST_GREEN];
		delta_joint[WRIST_GREEN] = saturate_round(NEXT_JOINT_STATE[WRIST_GREEN] - CURR_RAVEN_STATE.jpos[WRIST_GREEN], motor_maxspeed_rad);
		//GRASP1_GREEN
		NEXT_JOINT_STATE[GRASP1_GREEN]= (GreenSnake.mv(5) - calibrate_offset[GRASP1_GREEN])/calibrate_rate[GRASP1_GREEN];
		delta_joint[GRASP1_GREEN] = saturate_round(NEXT_JOINT_STATE[GRASP1_GREEN] - CURR_RAVEN_STATE.jpos[GRASP1_GREEN], motor_maxspeed_rad);
		//GRASP2_GREEN
		NEXT_JOINT_STATE[GRASP2_GREEN]= (GreenSnake.mv(6) - calibrate_offset[GRASP2_GREEN])/calibrate_rate[GRASP2_GREEN];
		delta_joint[GRASP2_GREEN] = saturate_round(NEXT_JOINT_STATE[GRASP2_GREEN] - CURR_RAVEN_STATE.jpos[GRASP2_GREEN], motor_maxspeed_rad);
	}
	else if (GreenSnake.m==1){
		//SHOULDER_GREEN
		NEXT_JOINT_STATE[SHOULDER_GREEN]= (GreenSnake.mv(0) - calibrate_offset[SHOULDER_GREEN])/calibrate_rate[SHOULDER_GREEN];
		delta_joint[SHOULDER_GREEN] = saturate_round(NEXT_JOINT_STATE[SHOULDER_GREEN] - CURR_RAVEN_STATE.jpos[SHOULDER_GREEN], motor_maxspeed_rad);
		//ELBOW_GREEN
		NEXT_JOINT_STATE[ELBOW_GREEN]= (GreenSnake.mv(1) - calibrate_offset[ELBOW_GREEN])/calibrate_rate[ELBOW_GREEN];
		delta_joint[ELBOW_GREEN] = saturate_round(NEXT_JOINT_STATE[ELBOW_GREEN] - CURR_RAVEN_STATE.jpos[ELBOW_GREEN], motor_maxspeed_rad);
		//Z_INS_GREEN
		NEXT_JOINT_STATE[Z_INS_GREEN]= (GreenSnake.mv(2) - calibrate_offset[Z_INS_GREEN])/calibrate_rate[Z_INS_GREEN];
		delta_joint[Z_INS_GREEN] = saturate_round(NEXT_JOINT_STATE[Z_INS_GREEN] - CURR_RAVEN_STATE.jpos[Z_INS_GREEN], motor_maxspeed_mm);
		//TOOL_ROT_GREEN
		NEXT_JOINT_STATE[TOOL_ROT_GREEN]= (GreenSnake.mv(3) - calibrate_offset[TOOL_ROT_GREEN])/calibrate_rate[TOOL_ROT_GREEN];
		delta_joint[TOOL_ROT_GREEN] = saturate_round(NEXT_JOINT_STATE[TOOL_ROT_GREEN] - CURR_RAVEN_STATE.jpos[TOOL_ROT_GREEN], motor_maxspeed_rad);
		//WRIST_GREEN
		NEXT_JOINT_STATE[WRIST_GREEN]= (GreenSnake.mv(4) - calibrate_offset[WRIST_GREEN])/calibrate_rate[WRIST_GREEN];
		delta_joint[WRIST_GREEN] = saturate_round(NEXT_JOINT_STATE[WRIST_GREEN] - CURR_RAVEN_STATE.jpos[WRIST_GREEN], motor_maxspeed_rad);
		//GRASP1_GREEN
		delta_joint[GRASP1_GREEN] = 0;
		//GRASP2_GREEN
		delta_joint[GRASP2_GREEN] = 0;
	}
	
}


/*
* \brief: based on a key, control each joint in the console in the following fashion
*
* \param int
*
* \return void
*/
void Raven_Controller::jointcontrol_keyboard_map(int theKey)
{
	float delta_rad = 0.01;
	float delta_mm = 0.001;
			switch(theKey)
			{
				//Gold ARM actions
				case '1':
				{
					//Up shoulder
					delta_joint[SHOULDER_GOLD]=delta_rad;
					cout<<"Moving SHOULDER_GOLD up"<<endl;
					break;
				}
				case 'q':
				{
					//Down shoulder
					delta_joint[SHOULDER_GOLD]=-delta_rad;
					cout<<"Moving SHOULDER_GOLD down"<<endl;
					break;
				}
				case '2':
				{
					//Up elbow
					delta_joint[ELBOW_GOLD]=delta_rad;
					cout<<"Moving ELBOW_GOLD up"<<endl;
					break;
				}
				case 'w':
				{
					//Down elbow
					delta_joint[ELBOW_GOLD]=-delta_rad;
					cout<<"Moving ELBOW_GOLD down"<<endl;
					break;
				}
				case '3':
				{
					//Up z_INS
					delta_joint[Z_INS_GOLD]=delta_mm;
					cout<<"Moving Z_INS_GOLD up"<<endl;
					break;
				}
				case 'e':
				{
					//Down Z_INS
					delta_joint[Z_INS_GOLD]=-delta_mm;
					cout<<"Moving Z_INS_GOLD down"<<endl;
					break;
				}
				case '4':
				{
					//Up TOOL_ROT_GOLD
					delta_joint[TOOL_ROT_GOLD]=delta_rad;
					cout<<"Moving TOOL_ROT_GOLD up"<<endl;
					break;
				}
				case 'r':
				{
					//down TOOL_ROT_GOLD
					delta_joint[TOOL_ROT_GOLD]=-delta_rad;
					cout<<"Moving TOOL_ROT_GOLD down"<<endl;
					break;
				}
				case '5':
				{
					//Up TOOL_ROT_GOLD
					delta_joint[WRIST_GOLD]=delta_rad;
					cout<<"Moving WRIST_GOLD up"<<endl;
					break;
				}
				case 't':
				{
					//down TOOL_ROT_GOLD
					delta_joint[WRIST_GOLD]=-delta_rad;
					cout<<"Moving WRIST_GOLD down"<<endl;
					break;
				}
				case '6':
				{
					//Up grasp 1
					delta_joint[GRASP1_GOLD]=delta_rad;
					cout<<"Moving GRASP1_GOLD up"<<endl;
					break;
				}
				case 'y':
				{
					//down grasp 1
					delta_joint[GRASP1_GOLD]=-delta_rad;
					cout<<"Moving GRASP1_GOLD down"<<endl;
					break;
				}
				case '7':
				{
					//Up grasp 2
					delta_joint[GRASP2_GOLD]=delta_rad;
					cout<<"Moving GRASP2_GOLD up"<<endl;
					break;
				}
				case 'u':
				{
					//down grasp 2
					delta_joint[GRASP2_GOLD]=-delta_rad;
					cout<<"Moving GRASP2_GOLD down"<<endl;
					break;
				}


				//GREEN ARM ACTIONS
				case 'a':
				{
					//Up shoulder
					delta_joint[SHOULDER_GREEN]=delta_rad;
					cout<<"Moving SHOULDER_GREEN up"<<endl;
					break;
				}
				case 'z':
				{
					//Down shoulder
					delta_joint[SHOULDER_GREEN]=-delta_rad;
					cout<<"Moving SHOULDER_GREEN down"<<endl;
					break;
				}
				case 's':
				{
					//Up elbow
					delta_joint[ELBOW_GREEN]=delta_rad;
					cout<<"Moving ELBOW_GREEN up"<<endl;
					break;
				}
				case 'x':
				{
					//Down elbow
					delta_joint[ELBOW_GREEN]=-delta_rad;
					cout<<"Moving ELBOW_GREEN down"<<endl;
					break;
				}
				case 'd':
				{
					//Up z_INS
					delta_joint[Z_INS_GREEN]=delta_mm;
					cout<<"Moving Z_INS_GREEN up"<<endl;
					break;
				}
				case 'c':
				{
					//Down Z_INS
					delta_joint[Z_INS_GREEN]=-delta_mm;
					cout<<"Moving Z_INS_GREEN down"<<endl;
					break;
				}
				case 'f':
				{
					//Up TOOL_ROT_GREEN
					delta_joint[TOOL_ROT_GREEN]=delta_rad;
					cout<<"Moving TOOL_ROT_GREEN up"<<endl;
					break;
				}
				case 'v':
				{
					//Down Z_INS
					delta_joint[TOOL_ROT_GREEN]=-delta_rad;
					cout<<"Moving TOOL_ROT_GREEN down"<<endl;
					break;
				}
				case 'g':
				{
					//Up WRIST
					delta_joint[WRIST_GREEN]=delta_rad;
					cout<<"Moving WRIST_GREEN up"<<endl;
					break;
				}
				case 'b':
				{
					//down wrist
					delta_joint[WRIST_GREEN]=-delta_rad;
					cout<<"Moving WRIST_GREEN down"<<endl;
					break;
				}
				case 'h':
				{
					//Up grasp 1
					delta_joint[GRASP1_GREEN]=delta_rad;
					cout<<"Moving GRASP1_GREEN up"<<endl;
					break;
				}
				case 'n':
				{
					//down grasp 1
					delta_joint[GRASP1_GREEN]=-delta_rad;
					cout<<"Moving GRASP1_GREEN down"<<endl;
					break;
				}
				case 'j':
				{
					//Up grasp 2
					delta_joint[GRASP2_GREEN]=delta_rad;
					cout<<"Moving GRASP2_GREEN up"<<endl;
					break;
				}
				case 'm':
				{
					//down grasp 2
					delta_joint[GRASP2_GREEN]=-delta_rad; 
					cout<<"Moving GRASP2_GREEN down"<<endl;
					break;
				}
	
			}
}
