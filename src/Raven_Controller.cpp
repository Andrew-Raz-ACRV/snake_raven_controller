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
	//Controller settings:
	this->dx_max = 2.0; //mm
	this->tol_error = 0.1; //magnitude error
	this->motor_maxspeed_rad1 = 0.0001/2.0; //rad for larger joints //5.0
	this->motor_maxspeed_rad2 = 0.001/2.0; //rad for smaller joints
	this->motor_maxspeed_mm = 0.0001/15.0; //mm

	this->PUB_COUNT = 0;
	this->SUB_COUNT = 0;
	this->counter = 0;
	this->factor = 0;

	this->RECEIVED_FIRST = false;
	this->SHOW_STATUS = false;
	this->NO_TARGET_SET = true;
	this->GO_HOME = true;
	this->CALIBRATED = false;
	this->MANUAL = false;

	//Initialise the arrays
	for(int i = 0; i < 14; i++){
		this->delta_joint[i] = 0;
		this->jpos_initial[i] = 0;
		this->mv2jpos_rate[i] = 1;
	}

	//Length of snake raven tool from mid-adaptor to Snake-base/RCM
	this->snake_length = 323.5;
	//NOTE default tool is 400mm long to RCM

	//Home Configuration for calibration
	//left to give way to right
	this->jpos_initial[SHOULDER_GOLD] = deg2rad(57); //deg2rad(39.5967);
	this->jpos_initial[ELBOW_GOLD] = deg2rad(-59); //deg2rad(-102.0840);
	this->jpos_initial[Z_INS_GOLD] = 500; //snake_length;

	//right to initial pose
	this->jpos_initial[SHOULDER_GREEN] = deg2rad(-39.5967);
	this->jpos_initial[ELBOW_GREEN] = deg2rad(-77.9160);
	this->jpos_initial[Z_INS_GREEN] = snake_length;

	//Calibration ratios for the tool:
	//converts mv into jpos 
	//left
	this->mv2jpos_rate[Z_INS_GOLD] = -1; //up is negative
	this->mv2jpos_rate[TOOL_ROT_GOLD] = -1.17;
	this->mv2jpos_rate[WRIST_GOLD] = 0.87;
	this->mv2jpos_rate[GRASP1_GOLD] = 0.87;
	this->mv2jpos_rate[GRASP2_GOLD] = -0.89;
	//right
	this->mv2jpos_rate[Z_INS_GREEN] = -1; //up is negative
	this->mv2jpos_rate[TOOL_ROT_GREEN] = -1.50; //p pan
	this->mv2jpos_rate[WRIST_GREEN] = 1.50; //p tilt
	this->mv2jpos_rate[GRASP1_GREEN] = -3.00; //-1.65; // d pan 2.25
	this->mv2jpos_rate[GRASP2_GREEN] = -3.00; //-1.55; // d tilt //3.00 is for extreme bending!

	//Old right arm rates
	//-1.50; //good for blue tool
	//0.90; //good
	//-0.87; //was inverted for blue tool
	//-0.91; //good

	//Set these things for SnakeRaven tools:
	this->GoldSnake.isrightarm = false;
	this->GreenSnake.isrightarm = true;
	this->Ttarget = GreenSnake.Tend;
	this->Tend = GreenSnake.Tend;

	//Record Data set up the columns for the csv file sec,nsec and each 12 values for Tend and Ttarget transforms
	this->RECORDING = true;
	this->record_freq = 100; //hz
	if (RECORDING){
		this->pFile = fopen("SnakeRavenData.csv","w");
		fprintf(pFile, "sec, nsec, Te(0 0), Tt(0 0), Te(1 0), Tt(1 0), Te(2 0), Tt(2 0), Te(0 1), Tt(0 1), Te(1 1), Tt(1 1), Te(2 1), Tt(2 1), Te(0 2), Tt(0 2), Te(1 2), Tt(1 2), Te(2 2), Tt(2 2), Te(0 3), Tt(0 3), Te(1 3), Tt(1 3), Te(2 3), Tt(2 3), q1, q1d, q2, q2d, q3, q3d, q4, q4d, q5, q5d, q6, q6d, q7, q7d,\n");
	}
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
	//Joint move
  	RavenJointmove_publisher = n.advertise<raven_jointmove>("raven_jointmove", 1);
	//Joint state!!
	RavenJoint_subscriber   = n.subscribe("joint_states", 1, &Raven_Controller::callback_joint_states,this);
	
	//Recording data publisher
	Snakeraven_publisher = n.advertise<snakeraven_state>("snakeraven_state", 1);
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
		cout<<"Calibration Begins..."<<endl;
		cout<<endl;
		cout<<"MOVING TO HOME CONFIGURATION..."<<endl;
		GO_HOME = true;
	}
	return false;
}

bool Raven_Controller::manual_calibration_words(bool print)  
{
	if(print)
	{
		cout<<endl;
		cout<<"Now check calibration manually..."<<endl;
		cout<<"Move joints manually to home configuration if it failed"<<endl;
		cout<<"Please put snake tool on robot if you haven't done so"<<endl;
		cout<<" -move the adpator joints with keyboard"<<endl;
		cout<<endl;
		cout<<"Snake tool should be completely perpendicular at RCM:      "<<endl;
		cout<<"-----------------------------------------------------------------"<<endl;
		cout<<"\t'1 2 3 4 5 6 7' : Increase Gold arm DOF"<<endl;
		cout<<"\t'q w e r t y u' : Decrease Gold arm DOF"<<endl;
		cout<<"\t'a s d f g h j' : Increase Green arm DOF"<<endl;
		cout<<"\t'z x c v b n m' : Decrease Green arm DOF"<<endl;
		cout<<"\t'k' : finish manual calibration"<<endl;
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
		int counts = 0;
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
						//change state to calibration
						state = calibration;
						print_menu = true;
					}
					else if(theKey=='1')
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

					//Move to initial configuration
					if (GO_HOME){
						if (move_2_home()==true){
							cout<<"\n\tFinished Move to home"<<endl;
							GO_HOME = false;
							MANUAL = true;
						}	
					}
					else{
						//Fine tune the robot manually, especially for tool placement
						MANUAL = manual_calibration_words(MANUAL);
						jointcontrol_keyboard_map(theKey);
					}

					//kill state
					if(theKey=='k')
					{
						//Go to calibrated motion mode
						//state = calibrated_motion; //DO Calibrated Motions
						state = idle; // SKIP CALIBRATED MOTIONS
						checkpoint = 0;
						print_menu = true;

						//Get current jpos as offset in jpos to q conversion
						calibrate_snake_raven_state();
						CALIBRATED = true;
						cout<<"\nCalibration Set!"<<endl;
						cout<<"\nNow testing Calibration..."<<endl;
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
						CALIBRATED = false; //must recalibrate if controlling joints individually
						cout<<"\nExiting mode..."<<endl;
					}
					break;
				}
				case calibrated_motion:
				{
					//For recording endeffector displacement ~works okay
					//Read current state Update Forward Kinematics
					//cout<<"stuck here"<<endl;
					updateSnakejoints();
					//cout<<"not stuck here"<<endl;
					GreenSnake.Tend = GreenSnake.FK();
					Tend = GreenSnake.Tend;
					if(RECORDING){
						//Assume the freq is 1000Hz, lets make it to record_freq 1000/10 = 100cylces/record:
						counts++;
						if(counts>(ROS_PUBLISH_RATE/record_freq)){
							record_to_csv();
							counts = 0;
						}
					}

					//For all checkpoints the raven joints are at initial state
					GreenSnake.q(0) = deg2rad(-39.5967);
					GreenSnake.q(1) = deg2rad(-77.9160);
					GreenSnake.q(2) = 0;

					//For all Desired Joint angles, make a ramp up and down factor
					
					if (counter<2000){
						//i.e. starts as 0 and increments  every iteration
						factor = factor + 0.00045; 
						//Will reach 0   +  0.00045*2000 = 0.9
					}
					else if(counter>4000){
						//Case when ramp down:
						factor = factor - 0.00045; 
						//Will reach 0.9  - 0.00045*2000 = 0.9-0.9 = 0
					}
					
					//factor = 0.9; //Original factor a constant

					//For single module:
					if(GreenSnake.m==1){
						//Move the snake tool to checkpoints to observe the calibration
						// the for rests all to 0, then it follows +pan -pan +tilt -tilt
						GreenSnake.q(3) = 0;
						GreenSnake.q(4) = 0;
						switch(checkpoint)
						{
							case 0:
								GreenSnake.q(3) = factor*GreenSnake.qu(3); //+pan
								break;
							case 1:
								GreenSnake.q(3) = factor*GreenSnake.ql(3); //-pan
								break;
							case 2:
								break; //break
							case 3:
								GreenSnake.q(4) = factor*GreenSnake.qu(4); //+tilt
								break;
							case 4:
								GreenSnake.q(4) = factor*GreenSnake.ql(4); //-tilt
								break;
							case 5:
								break; //break
							case 6:
								state = idle;
								print_menu = true;
								cout<<"\nCalibrated motions complete..."<<endl;
								cout<<"\nExiting mode..."<<endl;
								break;
						}
					}
					else{
						//Double Module Checkpoint
						GreenSnake.q(3) = 0;
						GreenSnake.q(4) = 0;
						GreenSnake.q(5) = 0;
						GreenSnake.q(6) = 0;

						switch(checkpoint)
						{
							case 0:
								GreenSnake.q(5) = factor*GreenSnake.qu(5); //+p pan
								break;
							case 1:
								GreenSnake.q(5) = factor*GreenSnake.ql(5); //-p pan
								break;
							case 2:
								break; //break
							case 3:
								GreenSnake.q(6) = factor*GreenSnake.qu(6); //+p tilt
								break;
							case 4:
								GreenSnake.q(6) = factor*GreenSnake.ql(6); //-p tilt
								break;
							case 5:
								break; //break
							case 12: //Stop here
								GreenSnake.q(5) = factor*GreenSnake.qu(5); //+d pan
								break;
							case 7:
								GreenSnake.q(5) = factor*GreenSnake.ql(5); //-d pan
								break;
							case 8:
								break; //break
							case 9:
								GreenSnake.q(6) = factor*GreenSnake.qu(6); //+d tilt
								break;
							case 10:
								GreenSnake.q(6) = factor*GreenSnake.ql(6); //-d tilt
								break;
							case 11:
								break; //break
							case 6:
								state = idle;
								print_menu = true;
								cout<<"\nCalibrated motions complete..."<<endl;
								cout<<"\nExiting mode..."<<endl;
								break;
						}
					}

					//Update Ttarget and record of desired q
					Ttarget = GreenSnake.FK();
					GreenSnake.q_desired = GreenSnake.q;

					//start message for the motion
					if (counter==0){
						cout<<"Checkpoint: "<<checkpoint<<endl;
						cout<<"Moving to configuration: "<<endl<<GreenSnake.q<<endl;
					}

					//Decide when to change checkpoints based on a counter of loops
					counter++;
					//cout<<counter<<endl;
					if (counter>6000){
						counter = 0;
						factor = 0;
						checkpoint++;
					}

					//Compute new Motor values
					GreenSnake.mv = GreenSnake.q2Motor_angles(); //Compute Motor values
					GoldSnake.mv = GoldSnake.q2Motor_angles(); //Compute Motor values

					updateRavenJointarray(); // send computed motor values

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
					Tend = GreenSnake.Tend;

					//Record Ttarget and Tend
					if(RECORDING){
						counts++;
						if(counts>(ROS_PUBLISH_RATE/record_freq)){
							record_to_csv();
							counts = 0;
						}
					}

					if (NO_TARGET_SET){
						Ttarget = GreenSnake.Tend;
						NO_TARGET_SET = false;
					}
					//cout << "key = " << theKey << endl;
					Ttarget = keyboard_teleop_map(theKey,Ttarget); //Ttarget update //GreenSnake.Tend

					dx = trans2dx(GreenSnake.Tend, Ttarget); //Update current Error
					
					cout << "Error = " << dx.norm() << endl;

					if (dx.norm() < tol_error) {
						cout << "Target Region Reached" << endl;
					}
					else {
						if (dx.norm() > dx_max) {
							dx = cap_mag(dx, dx_max); //Restrict delta size
						}
						//Update Jacobian, increment joint delta
						GreenSnake.J = GreenSnake.Jacobian(); //Update Jacobian

						//Weighted Damped Least Squares
						GreenSnake.q += dampedLeastSquares(GreenSnake.J, GreenSnake.q, GreenSnake.ql, GreenSnake.qu) * GreenSnake.W * dx;

						//Joint Limit check and saturation
						if (JointLimitcheck(GreenSnake.q, GreenSnake.ql, GreenSnake.qu)){
							cout << "\nWARNING: Workspace Limit Reached" << endl;
							//Saturate joint into jointspace
							GreenSnake.q = applyJointLimits(GreenSnake.q, GreenSnake.ql, GreenSnake.qu);
							//Move target back into workspace at that joint limit
							//GreenSnake.Tend = GreenSnake.FK();
							//Ttarget = GreenSnake.Tend;
						}
						//Update record of desired q joint value:
						GreenSnake.q_desired = GreenSnake.q;

						//Compute new Motor values
						GreenSnake.mv = GreenSnake.q2Motor_angles(); //Compute Motor values
						GoldSnake.mv = GoldSnake.q2Motor_angles(); //Compute Motor values

						updateRavenJointarray(); // send computed motor values

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
			publish_snakeraven_state(); //record data
			//output_SUBinfo();
			//output_PUBinfo();
		}

		if(RECEIVED_FIRST)
			cout<<"ros_process is shutdown."<<endl;
	}
	return 0;
}

//
// Move all joints to jpos_initial
//
bool Raven_Controller::move_2_home()
{
	bool reached = true; //assume it is there
	float error;
	//Check the joints and move
	for (int i = 0; i<14; i++){
		//check if the robot is there or not
		error = jpos_initial[i] - jpos_current[i];
		//send delta
		if ((i==Z_INS_GOLD)||(i==Z_INS_GREEN)){
			//prismatic
			if (reached && (fabs(error)<1)){
				reached = true;
			}
			else if (fabs(error)<1){
				delta_joint[i] = 0; //filters the jittering
			}
			else{
				reached = false;
				delta_joint[i] = saturate_round(error, motor_maxspeed_mm);
			}
		}
		else{
			//revolute
			if (reached && (fabs(error)<deg2rad(0.5))){
				reached = true;
			}
			else if (fabs(error)<deg2rad(0.5)){
				delta_joint[i] = 0; //filters the jittering
			}
			else{
				reached = false;
				delta_joint[i] = saturate_round(error, motor_maxspeed_rad1);
			}
		}
	}

	return reached;
}

/*
* \brief: puts the raven_state data into the two snakeraven class objects
* 
* \param void
*
* \return void
*/
void Raven_Controller::updateSnakejoints()
{
	//Use Calibration mv = jpos / mv2jpos + offset
	//Only the first 3 DOF because there is no reverse 
	//reading for tendon displacement to pan tilt angles
	for (int i = SHOULDER_GOLD; i<(SHOULDER_GOLD+GoldSnake.DOF); i++){
		//Left arm
		GoldSnake.mv_pre(i) = jpos_current[i] / mv2jpos_rate[i] + calibrate_offset[i];
	}

	for (int i = SHOULDER_GREEN; i<(SHOULDER_GREEN+GreenSnake.DOF); i++){
		//Right arm
		GreenSnake.mv_pre(i-SHOULDER_GREEN) = jpos_current[i] / mv2jpos_rate[i] + calibrate_offset[i];
	}

	//Convert the motor angles into pan and tilt joints
	GoldSnake.q = GoldSnake.Motor_angles2q();
	GreenSnake.q = GreenSnake.Motor_angles2q();

}

void Raven_Controller::updateRavenJointarray()
{
	//turn joint vector into a joint delta update
	//Cailbration linear function:	
	//jpos = (mv2jpos_rate[i]*(GreenSnake.mv(i-SHOULDER_GREEN) - calibrate_offset[i]))

	//Get jpos_desired initialiase as current jpos by default:
	for (int i = 0; i<14; i++){
		jpos_desired[i] = jpos_current[i];
	}

	//Extract motor values from snake raven class
	//Left arm
	for (int i = SHOULDER_GOLD; i < (SHOULDER_GOLD+GoldSnake.DOF); i++){
		jpos_desired[i] = (mv2jpos_rate[i]*(GoldSnake.mv(i-SHOULDER_GOLD) - calibrate_offset[i]));
	}
	//Right arm
	for (int i = SHOULDER_GREEN; i < (SHOULDER_GREEN+GreenSnake.DOF); i++){
		jpos_desired[i] = (mv2jpos_rate[i]*(GreenSnake.mv(i-SHOULDER_GREEN) - calibrate_offset[i]));
	}

	//Check the joints and move
	float error;
	for (int i = 0; i<14; i++){
		//check if the robot is there or not
		error = jpos_desired[i] - jpos_current[i];
		//send delta
		if ((i==Z_INS_GOLD)||(i==Z_INS_GREEN)){
			//prismatic
			if (fabs(error)<0.2){
				delta_joint[i] = 0; //filters the jittering
			}
			else{
				delta_joint[i] = saturate_round(error, motor_maxspeed_mm);
			}
		}
		else if ((i==SHOULDER_GOLD)||(i==ELBOW_GOLD)||(i==SHOULDER_GREEN)||(i==ELBOW_GREEN)){
			//slow revolute
			if (fabs(error)<deg2rad(0.5)){
				delta_joint[i] = 0; //filters the jittering
			}
			else{
				delta_joint[i] = saturate_round(error, motor_maxspeed_rad1);
			}
			//delta_joint[i] = saturate_round(error, motor_maxspeed_rad1);
		}
		else{
			//fast revolute
			if (fabs(error)<deg2rad(0.5)){
				delta_joint[i] = 0; //filters the jittering
			}
			else{
				delta_joint[i] = saturate_round(error, motor_maxspeed_rad2);
			}
		}

	}
}

//Record Function 

void Raven_Controller::publish_snakeraven_state()
{
	//We want to know the T_end and T_desired for snake raven, 
	//so convert that data and send it in a message
	static ros::Rate loop_rate(ROS_PUBLISH_RATE);
	static snakeraven_state msg_snakeraven_state;	

	// Put data into message
	msg_snakeraven_state.stamp = ros::Time::now(); //hdr

	//Convert Eigen matrices to 12 value arrays:
	int i = 0;
	for (int col = 0; col < 4; col++){
		for (int row = 0; row < 3; row++)
		{
			msg_snakeraven_state.Tend_desired[i] = Ttarget(row,col);
			msg_snakeraven_state.Tend_actual[i] = Tend(row,col);
			i++;
		}
	}

	// (2) send new command
	Snakeraven_publisher.publish(msg_snakeraven_state);
	ros::spinOnce();

	//(3) prepare for next publish
	loop_rate.sleep();
}

//csv record function
void Raven_Controller::record_to_csv()
{
	//We want to know the T_end and T_desired for snake raven, 
	//Time
	fprintf(pFile, "%"PRIu32", %"PRIu32",", ros::Time::now().sec, ros::Time::now().nsec);

	//Convert Eigen matrices to 12 values:
	for (int col = 0; col < 4; col++){
		for (int row = 0; row < 3; row++)
		{
			fprintf(pFile, "%.4f, %.4f,", Tend(row,col), Ttarget(row,col));
		}
	}

	//Put the Joint angles in the file as well, the measured q and the q_desired:
	for (int i = 0; i < GreenSnake.DOF; i++){
		fprintf(pFile, "%.4f, %.4f,", GreenSnake.q(i), GreenSnake.q_desired(i));
	}

	//End line
	fprintf(pFile, "\n");
}


//**********PRETTY MATURE FUNCTIONS DOWN HERE****//

/*
/ Brief: determines the offsets from home position set in the calibration mode
/
*/
void Raven_Controller::calibrate_snake_raven_state()
{
	//We know the current raven_state is equal to the initial q 
	//when this function is called in calibration
	//
	// CALIBRATION EQUATIONS
	// mv = jpos2mv * jpos + offset
	// mv = jpos / mv2jpos + offset
	//
	// offset = mv - jpos / mv2jpos
	//
	// jpos = mv2jpos * (mv - offset)

	//Joint values in snakeraven class reset to initial
	GreenSnake.reset_q();
	GoldSnake.reset_q();

	//Reset target 
	NO_TARGET_SET = true;	

	//Compute offset for Gold Snake
	for (int i = SHOULDER_GOLD; i<(SHOULDER_GOLD+GoldSnake.DOF); i++){
		calibrate_offset[i] = GoldSnake.mv(i) - jpos_current[i]/mv2jpos_rate[i];
	}

	//Compute offset for Green Snake
	for (int i = SHOULDER_GREEN; i<(SHOULDER_GREEN+GreenSnake.DOF); i++){
		calibrate_offset[i] = GreenSnake.mv(i-SHOULDER_GREEN) - jpos_current[i]/mv2jpos_rate[i];
	}	
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
	for (int i = 0; i<14; i++){
		msg_raven_jointmove.delta_joint[i] = delta_joint[i];
		//Reset delta_array to 0 for next loop
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
void Raven_Controller::callback_joint_states(sensor_msgs::JointState msg)
{
	// Get Joint Data from joint state
	//Reading joints like this: 0-6 left, 7-13 right
	//cout<<"Shoulder gold = "<<msg.position[0]<<endl;

	//(1) Copy joint state to local variable in radians and mm
	for (int i = 0; i<14; i++){
		if ((i==Z_INS_GOLD)||(i==Z_INS_GREEN)){
			jpos_current[i] = msg.position[i]; //mm
		}
		else{
			jpos_current[i] = deg2rad(msg.position[i]); //radians
		}
	}

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
	cout<<"snake_raven_controller publish: raven_jointmove["<<PUB_COUNT<<"]"<<endl;

	//Raven joint positions
		//Joint position update:
	for (int i=0; i<2; i++){
		//per arm
		if(i==0){
			cout<<"\n\tLEFT ARM delta_joint = "<<endl;	
			for (int j=0; j<7; j++){
				//Per DOF
				cout<<"\tDOF "<<j<<" = "<<delta_joint[i*SHOULDER_GREEN + j]<<endl;
			}
		}
		else if(i==1){
			cout<<"\n\tRIGHT ARM data1.position = "<<endl;
			for (int j=0; j<7; j++){
				//Per DOF
				cout<<"\tDOF "<<j<<" = "<<delta_joint[i*SHOULDER_GREEN + j]<<endl;
			}
		}
	}
	cout<<endl;
	cout<<endl;
	//ROS_INFO("snake_raven_controller publish: raven_jointmove[%d]", PUB_COUNT);
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
	cout<<"snake_raven_controller subscribe: joint_states["<<SUB_COUNT<<"]"<<endl;

	//Raven joint positions
	int value;
	//Joint position update:
	for (int i=0; i<2; i++){
		//per arm
		if(i==0){
			cout<<"\n\tLEFT_ARM.position = "<<endl;	
			for (int j=0; j<7; j++){
				//Per DOF
				value = i*SHOULDER_GREEN + j;

				if (value==Z_INS_GOLD){
					cout<<"\tDOF "<<j<<" = "<<jpos_current[value]<<endl;
				}
				else{
					cout<<"\tDOF "<<j<<" = "<<rad2deg(jpos_current[value])<<endl;
				}
			}
		}
		else if(i==1){
			cout<<"\n\tRIGHT_ARM.position = "<<endl;
			for (int j=0; j<7; j++){
				//Per DOF
				value = i*SHOULDER_GREEN + j;

				if (value==Z_INS_GREEN){
					cout<<"\tDOF "<<j<<" = "<<jpos_current[value]<<endl;
				}
				else{
					cout<<"\tDOF "<<j<<" = "<<rad2deg(jpos_current[value])<<endl;
				}
			}
		}
	}
	cout<<endl;
	cout<<endl;
}

double Raven_Controller::saturate_round(double input, float limit)
{
	double output;
	//round the input to a certain decimal place
	double value = round(input * 1000000); 

	output = value / 1000000.0;

	//saturate
	if (output>limit){
		output = limit;
	}
	else if (output<-limit){
		output = -limit;
	}

    return output;
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
	double delta = 0.1; //was 0.1mm
	double rot_delta = deg2rad(0.2); //rad was 0.2
	double pan_delta = deg2rad(5); //rad
	Matrix4d Ttarget = T;

	//theKey is -1 when no key is pressed, setting default to be no target doesn't really work
	//GreenSnake.q(5) = 0;
	//GreenSnake.q(6) = 0;

	switch(theKey)
	{
		//Teleoperation mapping
		case 's':
		{
			//Y--
			Ttarget(1, 3) -= delta;
			//cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case 'w':
		{
			//Y++
			Ttarget(1, 3) += delta;
			//cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case 'd':
		{
			//X++
			Ttarget(0, 3) += delta;
			//cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case 'a':
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
		case 'z':
		{
			//freeze motion
			NO_TARGET_SET = true;
			break;
		}
		case 'f':
		{
			//roty++
			//Affine3d Tr(AngleAxisd(rot_delta, Vector3d::UnitY()));
			//Ttarget = Ttarget * Tr.matrix();
			//Alternative distal pan --
			GreenSnake.q(5) -= pan_delta;
			Ttarget = GreenSnake.FK();
			break;
		}
		case 'h':
		{
			//roty--
			//Affine3d Tr(AngleAxisd(-rot_delta, Vector3d::UnitY()));
			//Ttarget = Ttarget * Tr.matrix();
			//Alternative distal pan ++
			GreenSnake.q(5) += pan_delta;
			Ttarget = GreenSnake.FK();
			break;
		}
		case 't':
		{
			//rotx+
			//Affine3d Tr(AngleAxisd(rot_delta, Vector3d::UnitX()));
			//Ttarget = Ttarget * Tr.matrix();
			//Alternative distal tilt --
			GreenSnake.q(6) -= pan_delta;
			Ttarget = GreenSnake.FK();
			break;
		}
		case 'g':
		{
			//rotx-
			//Affine3d Tr(AngleAxisd(-rot_delta, Vector3d::UnitX()));
			//Ttarget = Ttarget * Tr.matrix();
			//Alternative distal tilt ++
			GreenSnake.q(6) += pan_delta;
			Ttarget = GreenSnake.FK();
			break;
		}
		case 'y':
		{
			//rotz+
			Affine3d Tr(AngleAxisd(rot_delta, Vector3d::UnitZ()));
			Ttarget = Ttarget * Tr.matrix();
			break;
		}		
		case 'r':
		{
			//rotz-
			Affine3d Tr(AngleAxisd(-rot_delta, Vector3d::UnitZ()));
			Ttarget = Ttarget * Tr.matrix();
			break;
		}

	}
	return Ttarget;
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

/*
							case 0:	
								for (int i = 3; i<GreenSnake.DOF; i++){GreenSnake.q(i) = 0;}
								break;
							case 1:
								for (int i = 3; i<GreenSnake.DOF; i++){GreenSnake.q(i) = 0;}
								GreenSnake.q(3) = 0.9*GreenSnake.qu(3);
								break;
							case 2:	
								for (int i = 3; i<GreenSnake.DOF; i++){GreenSnake.q(i) = 0;}
								break;
							case 3:
								for (int i = 3; i<GreenSnake.DOF; i++){GreenSnake.q(i) = 0;}
								GreenSnake.q(3) = 0.9*GreenSnake.ql(3);
								break;
							case 4:
								for (int i = 3; i<GreenSnake.DOF; i++){GreenSnake.q(i) = 0;}
								break;
							case 5:
								for (int i = 3; i<GreenSnake.DOF; i++){GreenSnake.q(i) = 0;}
								GreenSnake.q(4) = 0.9*GreenSnake.qu(4);
								break;
							case 6:
								for (int i = 3; i<GreenSnake.DOF; i++){GreenSnake.q(i) = 0;}
								break;
							case 7:
								for (int i = 3; i<GreenSnake.DOF; i++){GreenSnake.q(i) = 0;}
								GreenSnake.q(4) = 0.9*GreenSnake.ql(4);
								break;
							case 8:
								for (int i = 3; i<GreenSnake.DOF; i++){GreenSnake.q(i) = 0;}
								break;							
							case 9:
								state = idle;
								print_menu = true;
								cout<<"\nCalibrated motions complete..."<<endl;
								cout<<"\nExiting mode..."<<endl;
								break;
								*/
