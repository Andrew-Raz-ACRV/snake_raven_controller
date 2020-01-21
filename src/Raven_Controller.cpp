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

	this->RECEIVED_FIRST = false;
	this->SHOW_STATUS = false;
	this->PAUSE = false;	
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
  	RavenAutomove_publisher = n.advertise<raven_automove>("raven_automove", 1);
	RavenState_subscriber   = n.subscribe("raven_state",1,&Raven_Controller::callback_raven_state,this);

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
				
			//State machine	
			switch(state)
			{
				case idle:
				{
					print_menu = menu_words(print_menu);
					//check theKey to see if the state changes
					if(theKey=='1')
					{
						//change state to joint control
						state = joint_control;
						print_menu = true;
					}
					else if(theKey=='2')
					{
						//change state to teleop control
						state = snake_teleop;	
						print_menu = true;
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

					//Update desired pose
					keyboard_teleop_map(theKey);

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
			//INITIALISE THE TARGET GOLD ARM ONLY
			updateSnakejoints();
			Ttarget = GoldSnake.FK();
			dx_max = 2;
		}

		while(ros::ok() && RECEIVED_FIRST)
		{
			if(state==snake_teleop)
			{
				// (1) Compute Forward Kinematics and Error

				//Joint vector from raven state
				updateSnakejoints();

				//Forward Kinematics
				GoldSnake.Tend = GoldSnake.FK();

				//Current Transform error
				dx = trans2dx(GoldSnake.Tend, Ttarget);

				//(2) Inverse Kinematics determine new joint update

				//Restrict delta size
				if (dx.norm() > dx_max) {
					dx = cap_mag(dx, dx_max);
				}
				//Update Jacobian
				GoldSnake.J = GoldSnake.Jacobian(); 
				//Joint Delta update
				GoldSnake.q += dampedLeastSquares(GoldSnake.J, GoldSnake.q, GoldSnake.ql, GoldSnake.qu) * dx;
				//Obey joint limits
				GoldSnake.q = applyJointLimits(GoldSnake.q, GoldSnake.ql, GoldSnake.qu);
				//Compute Motor values from tendon lengths
				GoldSnake.mv = GoldSnake.q2Motor_angles(); 

				// (3) Update raven state
				updateRavenState(); // send computed motor values into raven state				
			}

			// (4) publish new raven state command (send it out)

			//cout<<"Publishing raven_automove... "<<endl;
			publish_raven_automove();
			//output_PUBinfo();

		}

		if(RECEIVED_FIRST)
			cout<<"ros_process is shutdown."<<endl;
	}
	return 0;
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
void Raven_Controller::publish_raven_automove()
{
	static ros::Rate loop_rate(ROS_PUBLISH_RATE);
	static raven_automove msg_raven_automove;	

	// (1) wrap up the new command	
	msg_raven_automove.hdr.stamp = msg_raven_automove.hdr.stamp.now(); //hdr

	//JOINT POSITION PUBLISH
	//Left arm
	msg_raven_automove.joint_pos_d[SHOULDER_GOLD] = CURR_RAVEN_STATE.jpos_d[SHOULDER_GOLD];
	msg_raven_automove.joint_pos_d[ELBOW_GOLD] = CURR_RAVEN_STATE.jpos_d[ELBOW_GOLD];
	msg_raven_automove.joint_pos_d[Z_INS_GOLD] = CURR_RAVEN_STATE.jpos_d[Z_INS_GOLD];
	msg_raven_automove.joint_pos_d[TOOL_ROT_GOLD] = CURR_RAVEN_STATE.jpos_d[TOOL_ROT_GOLD];
	msg_raven_automove.joint_pos_d[WRIST_GOLD] = CURR_RAVEN_STATE.jpos_d[WRIST_GOLD];
	msg_raven_automove.joint_pos_d[GRASP1_GOLD] = CURR_RAVEN_STATE.jpos_d[GRASP1_GOLD];
	msg_raven_automove.joint_pos_d[GRASP2_GOLD] = CURR_RAVEN_STATE.jpos_d[GRASP2_GOLD];

	//Right arm
	msg_raven_automove.joint_pos_d[SHOULDER_GREEN] = CURR_RAVEN_STATE.jpos_d[SHOULDER_GREEN];
	msg_raven_automove.joint_pos_d[ELBOW_GREEN] = CURR_RAVEN_STATE.jpos_d[ELBOW_GREEN];
	msg_raven_automove.joint_pos_d[Z_INS_GREEN] = CURR_RAVEN_STATE.jpos_d[Z_INS_GREEN];
	msg_raven_automove.joint_pos_d[TOOL_ROT_GREEN] = CURR_RAVEN_STATE.jpos_d[TOOL_ROT_GREEN];
	msg_raven_automove.joint_pos_d[WRIST_GREEN] = CURR_RAVEN_STATE.jpos_d[WRIST_GREEN];
	msg_raven_automove.joint_pos_d[GRASP1_GREEN] = CURR_RAVEN_STATE.jpos_d[GRASP1_GREEN];
	msg_raven_automove.joint_pos_d[GRASP2_GREEN] = CURR_RAVEN_STATE.jpos_d[GRASP2_GREEN];

	// (2) send new command
	RavenAutomove_publisher.publish(msg_raven_automove);
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
	CURR_RAVEN_STATE.jpos_d[SHOULDER_GOLD] = msg.jpos_d[SHOULDER_GOLD];
	CURR_RAVEN_STATE.jpos_d[ELBOW_GOLD] = msg.jpos_d[ELBOW_GOLD];
	CURR_RAVEN_STATE.jpos_d[Z_INS_GOLD] = msg.jpos_d[Z_INS_GOLD];
	CURR_RAVEN_STATE.jpos_d[TOOL_ROT_GOLD] = msg.jpos_d[TOOL_ROT_GOLD];
	CURR_RAVEN_STATE.jpos_d[WRIST_GOLD] = msg.jpos_d[WRIST_GOLD];
	CURR_RAVEN_STATE.jpos_d[GRASP1_GOLD] = msg.jpos_d[GRASP1_GOLD];
	CURR_RAVEN_STATE.jpos_d[GRASP2_GOLD] = msg.jpos_d[GRASP2_GOLD];
	//Right arm
	CURR_RAVEN_STATE.jpos_d[SHOULDER_GREEN] = msg.jpos_d[SHOULDER_GREEN];
	CURR_RAVEN_STATE.jpos_d[ELBOW_GREEN] = msg.jpos_d[ELBOW_GREEN];
	CURR_RAVEN_STATE.jpos_d[Z_INS_GREEN] = msg.jpos_d[Z_INS_GREEN];
	CURR_RAVEN_STATE.jpos_d[TOOL_ROT_GREEN] = msg.jpos_d[TOOL_ROT_GREEN];
	CURR_RAVEN_STATE.jpos_d[WRIST_GREEN] = msg.jpos_d[WRIST_GREEN];
	CURR_RAVEN_STATE.jpos_d[GRASP1_GREEN] = msg.jpos_d[GRASP1_GREEN];
	CURR_RAVEN_STATE.jpos_d[GRASP2_GREEN] = msg.jpos_d[GRASP2_GREEN];

	// (2) update recieved data count
	SUB_COUNT ++;
	RECEIVED_FIRST = true;
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
	ROS_INFO("snake_raven_controller publish: raven_automove[%d]", PUB_COUNT);
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

	//Put CURR_RAVEN_STATE into Snake Raven joint vector objects
	GoldSnake.q(0) = deg2rad(CURR_RAVEN_STATE.jpos_d[SHOULDER_GOLD]);
	GoldSnake.q(1) = deg2rad(CURR_RAVEN_STATE.jpos_d[ELBOW_GOLD]);
	GoldSnake.q(2) = CURR_RAVEN_STATE.jpos_d[Z_INS_GOLD];
	//GoldSnake.q(3) = deg2rad(CURR_RAVEN_STATE.jpos_d[TOOL_ROT_GOLD]);
	//GoldSnake.q(4) = deg2rad(CURR_RAVEN_STATE.jpos_d[WRIST_GOLD]);
	//GoldSnake.q(5) = deg2rad(CURR_RAVEN_STATE.jpos_d[GRASP1_GOLD]);
	//GoldSnake.q(6) = deg2rad(CURR_RAVEN_STATE.jpos_d[GRASP2_GOLD]);

	GreenSnake.q(0) = deg2rad(CURR_RAVEN_STATE.jpos_d[SHOULDER_GREEN]);
	GreenSnake.q(1) = deg2rad(CURR_RAVEN_STATE.jpos_d[ELBOW_GREEN]);
	GreenSnake.q(2) = CURR_RAVEN_STATE.jpos_d[Z_INS_GREEN];
	//GreenSnake.q(3) = deg2rad(CURR_RAVEN_STATE.jpos_d[TOOL_ROT_GREEN]);
	//GreenSnake.q(4) = deg2rad(CURR_RAVEN_STATE.jpos_d[WRIST_GREEN]);
	//GreenSnake.q(5) = deg2rad(CURR_RAVEN_STATE.jpos_d[GRASP1_GREEN]);
	//GreenSnake.q(6) = deg2rad(CURR_RAVEN_STATE.jpos_d[GRASP2_GREEN]);

	//Note that there is no reverse mapping from Raven motor value to pan and tilt
	//assume initially zeros from snakeRaven constructor
}

void Raven_Controller::updateRavenState()
{
	//turn joint vector into desired joint position in udpatecurr_raven_state
	//Left arm
	CURR_RAVEN_STATE.jpos_d[SHOULDER_GOLD] = rad2deg(GoldSnake.mv(0));
	CURR_RAVEN_STATE.jpos_d[ELBOW_GOLD] = rad2deg(GoldSnake.mv(1));
	CURR_RAVEN_STATE.jpos_d[Z_INS_GOLD] = GoldSnake.mv(2);
	CURR_RAVEN_STATE.jpos_d[TOOL_ROT_GOLD] = rad2deg(GoldSnake.mv(3));
	CURR_RAVEN_STATE.jpos_d[WRIST_GOLD] = rad2deg(GoldSnake.mv(4));
	CURR_RAVEN_STATE.jpos_d[GRASP1_GOLD] = rad2deg(GoldSnake.mv(5));
	CURR_RAVEN_STATE.jpos_d[GRASP2_GOLD] = rad2deg(GoldSnake.mv(6));

	//Right arm
	CURR_RAVEN_STATE.jpos_d[SHOULDER_GREEN] = rad2deg(GreenSnake.mv(0));
	CURR_RAVEN_STATE.jpos_d[ELBOW_GREEN] = rad2deg(GreenSnake.mv(1));
	CURR_RAVEN_STATE.jpos_d[Z_INS_GREEN] = GreenSnake.mv(2);
	CURR_RAVEN_STATE.jpos_d[TOOL_ROT_GREEN] = rad2deg(GreenSnake.mv(3));
	CURR_RAVEN_STATE.jpos_d[WRIST_GREEN] = rad2deg(GreenSnake.mv(4));
	CURR_RAVEN_STATE.jpos_d[GRASP1_GREEN] = rad2deg(GreenSnake.mv(5));
	CURR_RAVEN_STATE.jpos_d[GRASP2_GREEN] = rad2deg(GreenSnake.mv(6));		
	
}

/*
* \brief: based on a key, control the target in the console in the following fashion
*
* \param int
*
* \return void
*/
void Raven_Controller::keyboard_teleop_map(int theKey)
{
	//Keyboard update step
	double delta = 1;

	switch(theKey)
	{
		//Teleoperation mapping
		case 'a':
		{
			//Y--
			Ttarget(1, 3) -= delta;
			cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case 'd':
		{
			//Y++
			Ttarget(1, 3) += delta;
			cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case 'w':
		{
			//X++
			Ttarget(0, 3) += delta;
			cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case 's':
		{
			//X--
			Ttarget(0, 3) -= delta;
			cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case 'q':
		{
			//Z++
			Ttarget(2, 3) += delta;
			cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case 'e':
		{
			//Z--
			Ttarget(2, 3) -= delta;
			cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case 'z':
		{
			//Make Target the current forward kinematics
			updateSnakejoints();
			cout<<"\nRESETTING TARGET\n"<<endl;
			cout << "Target= " << endl << Ttarget << endl;
			Ttarget = GoldSnake.FK();
			break;
		}
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
			switch(theKey)
			{
				//Gold ARM actions
				case '1':
				{
					//Up shoulder
					CURR_RAVEN_STATE.jpos_d[SHOULDER_GOLD]++;
					cout<<"Moving SHOULDER_GOLD up"<<endl;
					break;
				}
				case 'q':
				{
					//Down shoulder
					CURR_RAVEN_STATE.jpos_d[SHOULDER_GOLD]--;
					cout<<"Moving SHOULDER_GOLD down"<<endl;
					break;
				}
				case '2':
				{
					//Up elbow
					CURR_RAVEN_STATE.jpos_d[ELBOW_GOLD]++;
					cout<<"Moving ELBOW_GOLD up"<<endl;
					break;
				}
				case 'w':
				{
					//Down elbow
					CURR_RAVEN_STATE.jpos_d[ELBOW_GOLD]--;
					cout<<"Moving ELBOW_GOLD down"<<endl;
					break;
				}
				case '3':
				{
					//Up z_INS
					CURR_RAVEN_STATE.jpos_d[Z_INS_GOLD]++;
					cout<<"Moving Z_INS_GOLD up"<<endl;
					break;
				}
				case 'e':
				{
					//Down Z_INS
					CURR_RAVEN_STATE.jpos_d[Z_INS_GOLD]--;
					cout<<"Moving Z_INS_GOLD down"<<endl;
					break;
				}
				case '4':
				{
					//Up TOOL_ROT_GOLD
					CURR_RAVEN_STATE.jpos_d[TOOL_ROT_GOLD]++;
					cout<<"Moving TOOL_ROT_GOLD up"<<endl;
					break;
				}
				case 'r':
				{
					//down TOOL_ROT_GOLD
					CURR_RAVEN_STATE.jpos_d[TOOL_ROT_GOLD]--;
					cout<<"Moving TOOL_ROT_GOLD down"<<endl;
					break;
				}
				case '5':
				{
					//Up TOOL_ROT_GOLD
					CURR_RAVEN_STATE.jpos_d[WRIST_GOLD]++;
					cout<<"Moving WRIST_GOLD up"<<endl;
					break;
				}
				case 't':
				{
					//down TOOL_ROT_GOLD
					CURR_RAVEN_STATE.jpos_d[WRIST_GOLD]--;
					cout<<"Moving WRIST_GOLD down"<<endl;
					break;
				}
				case '6':
				{
					//Up grasp 1
					CURR_RAVEN_STATE.jpos_d[GRASP1_GOLD]++;
					cout<<"Moving GRASP1_GOLD up"<<endl;
					break;
				}
				case 'y':
				{
					//down grasp 1
					CURR_RAVEN_STATE.jpos_d[GRASP1_GOLD]--;
					cout<<"Moving GRASP1_GOLD down"<<endl;
					break;
				}
				case '7':
				{
					//Up grasp 2
					CURR_RAVEN_STATE.jpos_d[GRASP2_GOLD]++;
					cout<<"Moving GRASP2_GOLD up"<<endl;
					break;
				}
				case 'u':
				{
					//down grasp 2
					CURR_RAVEN_STATE.jpos_d[GRASP2_GOLD]--;
					cout<<"Moving GRASP2_GOLD down"<<endl;
					break;
				}


				//GREEN ARM ACTIONS
				case 'a':
				{
					//Up shoulder
					CURR_RAVEN_STATE.jpos_d[SHOULDER_GREEN]++;
					cout<<"Moving SHOULDER_GREEN up"<<endl;
					break;
				}
				case 'z':
				{
					//Down shoulder
					CURR_RAVEN_STATE.jpos_d[SHOULDER_GREEN]--;
					cout<<"Moving SHOULDER_GREEN down"<<endl;
					break;
				}
				case 's':
				{
					//Up elbow
					CURR_RAVEN_STATE.jpos_d[ELBOW_GREEN]++;
					cout<<"Moving ELBOW_GREEN up"<<endl;
					break;
				}
				case 'x':
				{
					//Down elbow
					CURR_RAVEN_STATE.jpos_d[ELBOW_GREEN]--;
					cout<<"Moving ELBOW_GREEN down"<<endl;
					break;
				}
				case 'd':
				{
					//Up z_INS
					CURR_RAVEN_STATE.jpos_d[Z_INS_GREEN]++;
					cout<<"Moving Z_INS_GREEN up"<<endl;
					break;
				}
				case 'c':
				{
					//Down Z_INS
					CURR_RAVEN_STATE.jpos_d[Z_INS_GREEN]--;
					cout<<"Moving Z_INS_GREEN down"<<endl;
					break;
				}
				case 'f':
				{
					//Up TOOL_ROT_GREEN
					CURR_RAVEN_STATE.jpos_d[TOOL_ROT_GREEN]++;
					cout<<"Moving TOOL_ROT_GREEN up"<<endl;
					break;
				}
				case 'v':
				{
					//Down Z_INS
					CURR_RAVEN_STATE.jpos_d[TOOL_ROT_GREEN]--;
					cout<<"Moving TOOL_ROT_GREEN down"<<endl;
					break;
				}
				case 'g':
				{
					//Up WRIST
					CURR_RAVEN_STATE.jpos_d[WRIST_GREEN]++;
					cout<<"Moving WRIST_GREEN up"<<endl;
					break;
				}
				case 'b':
				{
					//down wrist
					CURR_RAVEN_STATE.jpos_d[WRIST_GREEN]--;
					cout<<"Moving WRIST_GREEN down"<<endl;
					break;
				}
				case 'h':
				{
					//Up grasp 1
					CURR_RAVEN_STATE.jpos_d[GRASP1_GREEN]++;
					cout<<"Moving GRASP1_GREEN up"<<endl;
					break;
				}
				case 'n':
				{
					//down grasp 1
					CURR_RAVEN_STATE.jpos_d[GRASP1_GREEN]--;
					cout<<"Moving GRASP1_GREEN down"<<endl;
					break;
				}
				case 'j':
				{
					//Up grasp 2
					CURR_RAVEN_STATE.jpos_d[GRASP2_GREEN]++;
					cout<<"Moving GRASP2_GREEN up"<<endl;
					break;
				}
				case 'm':
				{
					//down grasp 2
					CURR_RAVEN_STATE.jpos_d[GRASP2_GREEN]--;
					cout<<"Moving GRASP2_GREEN down"<<endl;
					break;
				}
	
			}
}
