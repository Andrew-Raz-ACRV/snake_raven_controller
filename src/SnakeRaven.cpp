#include "SnakeRaven.h"

//Snake Raven Constructor Function: PUT PARAMETERS IN HERE
SnakeRaven::SnakeRaven() {
	//Design Parameter Construction
	// number of modules
	m = Max_m;

	//Degrees of Freedom:
	DOF = 3 + 2 * m;//number of DOF
	da = 10; //mm for adaptor capstan diameter
	q = MatrixXd::Zero(DOF, 1); // Joint vector

	//Parameter sets:
	if (m == 1) {
		//Single module
		alpha << 1.24; //radians
		n << 3; //integers
		d << 1.62; //mm
		//Initial configuration
		q << 0, 0, 0, 0, 0;
	}
	else if (m == 2) {
		//Double module
		alpha << 1.39, 1.18; //radians
		n << 1, 3; //integers
		d << 6, 0.41; //mm
		//Initial configuration
		q << 0, 0, 0, 0, 0, 0, 0;
		//q << 0, 0, 25, 0.2, 0.05, 0.4, 0.1;
	}
	
	//Other parameters
	w = 4;//mm
	isrightarm = true;

	//Tool Transform is a short z-axis translation
	Tool = Translation3d(0, 0, 5);

	//Get Neutral Tendon lengths
	dL0 = GetTendonLengths();

	//Calibration constants
	rate << MatrixXd::Ones(DOF, 1);
	offset << MatrixXd::Zero(DOF, 1);
	
	//Run all functions
	Tend = FK(); //initial tool pose
	J = Jacobian(); //Initial Jacobian matrix
	mv = q2Motor_angles(); //Current Motor values

	//Automate Joint limit definitions:

	//Joint Limit definition
	ql(0) = -PI / 2; ql(1) = -PI / 2; ql(2) = -50;//mm
	qu(0) = PI / 2; qu(1) = PI / 2; qu(2) = 200;//mm

	//Variable Module Joint limits:
	for (int i = 0; i < m; i++)
	{
		double theta_max = alpha(i)*n(i) / 2; //Maximum Bending in pan/tilt for module
		//Append pan and tilt limits:
		ql(3 + 2 * i) = -theta_max;		ql(4 + 2 * i) = -theta_max;
		qu(3 + 2 * i) = theta_max;		qu(4 + 2 * i) = theta_max;
	}

}

//Forward Kinematics Function
MatrixXd SnakeRaven::FK()
{
	//Start transform from Raven Base:
	double La12 = deg2rad(75); double La23 = deg2rad(52);
	Matrix4d RCM, T1, T2, T3;

	if (isrightarm == 1) {
		//Right arm
		RCM << 0, 0, -1, -300.71, 0, 1, 0, 61, 1, 0, 0, -7, 0, 0, 0, 1;
		T1 = dh(PI, 0, 0, q(0));
		T2 = dh(La12, 0, 0, q(1));
		T3 = dh(La23, 0, q(2), -PI / 2.0);
	}
	else {
		//Left arm
		RCM << 0, 0, 1, 300.71, 0, -1, 0, 61, 1, 0, 0, -7, 0, 0, 0, 1;
		T1 = dh(0, 0, 0, q(0));
		T2 = dh(La12, 0, 0, q(1));
		T3 = dh(PI - La23, 0, q(2), PI / 2.0);
	}
	//Start Kinematic Chain on Raven DOF
	Tend = RCM * T1*T2*T3;

	//Start snakebot Kinematic chain
	int first_disk = 1;

	//The pan first assumption
	bool pan_first = true;

	//Loop through each module k until m
	for (int k = 0; k < m; k++)
	{
		//Decide on number of pan and tilt joints:
		double np, nt;
		if (pan_first == true) {
			np = round(n(k) / 2.0); nt = n(k) - np;
		}
		else {
			nt = round(n(k) / 2.0); np = n(k) - nt;
		}

		//Extract joint angles for pan and tilt of module k
		double Op = q(3 + 2 * k); //pan angle
		double Ot = q(4 + 2 * k); //tilt angle

								  //Get Radius of curvature from alpha:
		double r = (w / 2.0) / sin(alpha(k));

		//Compute Pan transform:
		double r01x = 0;
		double r01y = -2 * r * (1 - cos(alpha(k))*cos(Op / (2 * np)))*sin(Op / (2 * np));
		double r01z = 2 * r * (1 - cos(alpha(k))*cos(Op / (2 * np)))*cos(Op / (2 * np));
		Affine3d T01(Translation3d(r01x, r01y, r01z) * AngleAxisd((Op / np), Vector3d::UnitX()));
		Affine3d T11(Translation3d(0, 0, d(k)));
		Affine3d Tpan = T01 * T11;

		//Compute Tilt Transform:
		double r12x = 2 * r * (1 - cos(alpha(k))*cos(Ot / (2 * nt)))*sin(Ot / (2 * nt));
		double r12y = 0;
		double r12z = 2 * r * (1 - cos(alpha(k))*cos(Ot / (2 * nt)))*cos(Ot / (2 * nt));
		Affine3d T12(Translation3d(r12x, r12y, r12z) * AngleAxisd((Ot / nt), Vector3d::UnitY()));
		Affine3d T22(Translation3d(0, 0, d(k)));
		Affine3d Ttilt = T12 * T22;

		//Kinematic Chain
		for (int i = first_disk; i <= (first_disk + n(k) - 1); i++)
		{
			if (pan_first == true) {
				if (isodd(i - (first_disk - 1))) {
					Tend = Tend * Tpan.matrix(); //pan
				}
				else {
					Tend = Tend * Ttilt.matrix(); //tilt
				}
			}
			else {
				if (isodd(i - (first_disk - 1))) {
					Tend = Tend * Ttilt.matrix(); //tilt
				}
				else {
					Tend = Tend * Tpan.matrix(); //pan
				}
			}
		}

		//Transition disk:
		if (k != (m - 1)) {
			Affine3d Tpd(AngleAxisd(deg2rad(-90 / m), Vector3d::UnitZ()));
			Tend = Tend * Tpd.matrix();
		}
		//Next section decide if it starts as a pan or tilt
		if ((pan_first == true) && (isodd(n(k)))) {
			pan_first = false;
		}
		else if ((pan_first == false) && (isodd(n(k)))) {
			pan_first = true;
		}
		else if ((pan_first == true) && (isodd(n(k)) == false)) {
			pan_first = true;
		}
		else if ((pan_first == false) && (isodd(n(k)) == false)) {
			pan_first = false;
		}
		//Update first disk for the next segment:
		first_disk = first_disk + static_cast<int>(n(k));
	}
	//Append Tool transform
	Tend = Tend * Tool.matrix();
	return Tend;
}

//Jacobian Function
MatrixXd SnakeRaven::Jacobian()
{
	//Initialise the output matrix 6 by DOF as zeros:
	MatrixXd J(6, DOF);
	J = MatrixXd::Zero(6, DOF);

	//Solve analytically by doing FK algorithm 
	//and measuring velocity of endeffector from rotation from each joint

	//Initialise measurement vectors:
	Vector3d Pg, P, wz; // endeffector Pg, point P, axis wz 

						//Endeffector Point:
	Pg = Tend.block(0, 3, 3, 1);

	//Start FK from Raven Base:
	double La12 = deg2rad(75); double La23 = deg2rad(52);
	Matrix4d T, RCM, T1, T2, T3;

	if (isrightarm == 1) {
		//Right arm
		RCM << 0, 0, -1, -300.71, 0, 1, 0, 61, 1, 0, 0, -7, 0, 0, 0, 1;
		T1 = dh(PI, 0, 0, q(0));
		T2 = dh(La12, 0, 0, q(1));
		T3 = dh(La23, 0, q(2), -PI / 2.0);
	}
	else {
		//Left arm
		RCM << 0, 0, 1, 300.71, 0, -1, 0, 61, 1, 0, 0, -7, 0, 0, 0, 1;
		T1 = dh(0, 0, 0, q(0));
		T2 = dh(La12, 0, 0, q(1));
		T3 = dh(PI - La23, 0, q(2), PI / 2.0);
	}

	//Start solving Raven Jacobian columns:
	//Jq1
	T = RCM * T1;
	P = T.block(0, 3, 3, 1);
	wz = T.block(0, 0, 3, 3) * Vector3d::UnitZ();
	J.block(0, 0, 3, 1) = wz.cross(Pg - P);
	J.block(3, 0, 3, 1) = wz;
	//Jq2
	T = T * T2;
	P = T.block(0, 3, 3, 1);
	wz = T.block(0, 0, 3, 3) * Vector3d::UnitZ();
	J.block(0, 1, 3, 1) = wz.cross(Pg - P);
	J.block(3, 1, 3, 1) = wz;
	//Jq3
	T = T * T3;
	P = T.block(0, 3, 3, 1);
	wz = T.block(0, 0, 3, 3) * Vector3d::UnitZ();
	J.block(0, 2, 3, 1) = wz;
	J.block(3, 2, 3, 1) = MatrixXd::Zero(3, 1);

	//Start solving Snakebot Jacobian columns
	int first_disk = 1; int coln;

	//The pan first assumption
	bool pan_first = true;

	//Loop through each module k until m
	for (int k = 0; k < m; k++)
	{
		//Get Radius of curvature from alpha:
		double r = (w / 2.0) / sin(alpha(k));

		//Decide on number of pan and tilt joints:
		double np, nt;
		if (pan_first == true) {
			np = round(n(k) / 2.0); nt = n(k) - np;
		}
		else {
			nt = round(n(k) / 2.0); np = n(k) - nt;
		}

		//Initialise the Apparent Jacobian Matrices for Pan and tilt for this module
		MatrixXd Ja_pan = MatrixXd::Zero(6, static_cast<int>(np));
		MatrixXd Ja_tilt = MatrixXd::Zero(6, static_cast<int>(nt));

		//Extract joint angles for pan and tilt of module k
		double Op = q(3 + 2 * k); //pan angle
		double Ot = q(4 + 2 * k); //tilt angle

								  //Compute Pan transform:
		double r01x = 0;
		double r01y = -2 * r * (1 - cos(alpha(k))*cos(Op / (2 * np)))*sin(Op / (2 * np));
		double r01z = 2 * r * (1 - cos(alpha(k))*cos(Op / (2 * np)))*cos(Op / (2 * np));
		Affine3d T01(Translation3d(r01x, r01y, r01z) * AngleAxisd((Op / np), Vector3d::UnitX()));
		Affine3d T11(Translation3d(0, 0, d(k)));
		Affine3d Tpan = T01 * T11;

		//Compute Tilt Transform:
		double r12x = 2 * r * (1 - cos(alpha(k))*cos(Ot / (2 * nt)))*sin(Ot / (2 * nt));
		double r12y = 0;
		double r12z = 2 * r * (1 - cos(alpha(k))*cos(Ot / (2 * nt)))*cos(Ot / (2 * nt));
		Affine3d T12(Translation3d(r12x, r12y, r12z) * AngleAxisd((Ot / nt), Vector3d::UnitY()));
		Affine3d T22(Translation3d(0, 0, d(k)));
		Affine3d Ttilt = T12 * T22;

		//Kinematic Chain
		for (int i = first_disk; i <= (first_disk + n(k) - 1); i++)
		{
			if (pan_first == true) {
				if (isodd(i - (first_disk - 1))) {
					T = T * Tpan.matrix(); //pan
					P = T.block(0, 3, 3, 1); wz = T.block(0, 0, 3, 3) * Vector3d::UnitX();
					coln = static_cast<int>(ceil((i - (first_disk - 1)) / 2.0)) - 1;
					Ja_pan.block(0, coln, 3, 1) = wz.cross(Pg - P);
					Ja_pan.block(3, coln, 3, 1) = wz;
				}
				else {
					T = T * Ttilt.matrix(); //tilt
					P = T.block(0, 3, 3, 1); wz = T.block(0, 0, 3, 3) * Vector3d::UnitY();
					coln = static_cast<int>(((i - (first_disk - 1)) / 2.0)) - 1;
					Ja_tilt.block(0, coln, 3, 1) = wz.cross(Pg - P);
					Ja_tilt.block(3, coln, 3, 1) = wz;
				}
			}
			else {
				if (isodd(i - (first_disk - 1))) {
					T = T * Ttilt.matrix(); //tilt
					P = T.block(0, 3, 3, 1); wz = T.block(0, 0, 3, 3) * Vector3d::UnitY();
					coln = static_cast<int>(ceil((i - (first_disk - 1)) / 2.0)) - 1;
					Ja_tilt.block(0, coln, 3, 1) = wz.cross(Pg - P);
					Ja_tilt.block(3, coln, 3, 1) = wz;
				}
				else {
					T = T * Tpan.matrix(); //pan
					P = T.block(0, 3, 3, 1); wz = T.block(0, 0, 3, 3) * Vector3d::UnitX();
					coln = static_cast<int>(((i - (first_disk - 1)) / 2.0)) - 1;
					Ja_pan.block(0, coln, 3, 1) = wz.cross(Pg - P);
					Ja_pan.block(3, coln, 3, 1) = wz;
				}
			}
		}

		//Reduce Apparent Pan and tilt Jacobians to their single degree of freedom
		if (np != 0) {
			J.block(0, 3 + 2 * k, 6, 1) = Ja_pan.rowwise().sum() / np;
		}
		if (nt != 0) {
			J.block(0, 4 + 2 * k, 6, 1) = Ja_tilt.rowwise().sum() / nt;
		}

		//Transition disk:
		if (k != (m - 1)) {
			Affine3d Tpd(AngleAxisd(deg2rad(-90 / m), Vector3d::UnitZ()));
			T = T * Tpd.matrix();
		}
		//Next section decide if it starts as a pan or tilt
		if ((pan_first == true) && (isodd(n(k)))) {
			pan_first = false;
		}
		else if ((pan_first == false) && (isodd(n(k)))) {
			pan_first = true;
		}
		else if ((pan_first == true) && (isodd(n(k)) == false)) {
			pan_first = true;
		}
		else if ((pan_first == false) && (isodd(n(k)) == false)) {
			pan_first = false;
		}
		//Update first disk for the next segment:
		first_disk = first_disk + static_cast<int>(n(k));
	}
	return J;
}

// TENDON CALCULATION FUNCTIONS 

//Tendon point function
MatrixXd GetTendonPoints(const MatrixXd& T, const double w, const double alpha, const double d, bool B, bool pan, const int k, const int M) {
	// T is the transform
	// w is the width
	// alpha is half the contact angle
	// d is the distance between curved surfaces
	// B is a logic true for end or false for start
	// pan is a logic for pan or tilt
	// k is the current segment/module e.g. 0 or 1
	// M is the total number of segments in system e.g. 2

	//Initialise output: 4 tendon points (pl pr tl tr) per module, x y z columns
	MatrixXd p = MatrixXd::Zero(4 * M, 3);

	//Get Width Radius and curve radius
	double rad = w / 2.0;
	double r = rad / sin(alpha);
	double theta;

	//Define Tendon Placement angles:
	Vector4d angles; RowVector3d po;
	// Tendon order: Pan left (-90), Pan right (90), tilt left (0), tilt right (180)
	angles << -PI / 2.0, PI / 2.0, 0, PI;

	if (pan == 1) { //pan case
		if (B == 0) { //A beginning rolling joint frame
			for (int i = k+1; i <= M; i++)
			{
				//Go around the circle and get points
				for (int j = 0; j < 4; j++)
				{
					theta = angles(j) + (i - k - 1)*deg2rad(-90 / M);
					po << rad * cos(theta), rad*sin(theta), sqrt(pow(r,2.0) - pow(rad*sin(theta),2.0)) - r * cos(alpha);
					p.block((i - 1) * 4 + j, 0, 1, 3) = po;
				}
			}
		}
		else if (B == 1) {
			for (int i = k+1; i <= M; i++)
			{
				//Go around the circle and get points
				for (int j = 0; j < 4; j++)
				{
					theta = angles(j) + (i - k - 1)*deg2rad(-90 / M);
					po << rad * cos(theta), rad*sin(theta), -sqrt(pow(r, 2.0) - pow(rad*sin(theta), 2.0)) + r * cos(alpha) - d;
					p.block((i - 1) * 4 + j, 0, 1, 3) = po;
				}
			}
		}
	}
	else if (pan == 0) { //tilt case
		if (B == 0) {
			for (int i = k+1; i <= M; i++)
			{
				//Go around the circle and get points
				for (int j = 0; j < 4; j++)
				{
					theta = angles(j) + (i - k - 1)*deg2rad(-90 / M);
					po << rad * cos(theta), rad*sin(theta), sqrt(pow(r, 2.0) - pow(rad*cos(theta), 2.0)) - r * cos(alpha);
					p.block((i - 1) * 4 + j, 0, 1, 3) = po;
				}
			}
		}
		else if (B == 1) {
			for (int i = k+1; i <= M; i++)
			{
				//Go around the circle and get points
				for (int j = 0; j < 4; j++)
				{
					theta = angles(j) + (i - k - 1)*deg2rad(-90 / M);
					po << rad * cos(theta), rad*sin(theta), -sqrt(pow(r, 2.0) - pow(rad*cos(theta), 2.0)) + r * cos(alpha) - d;
					p.block((i - 1) * 4 + j, 0, 1, 3) = po;
				}
			}
		}
	}

	//Determine Null points (when segment 1 finishes its still 0 0 0)
	MatrixXd Logic(p.rows(), 3);
	Logic = (p.rowwise()).any();

	//Transform to coordinate frame T
	p = TransformPoints(T,p);

	//Mask out the null points after the transformation
	for (int i = 0; i < p.rows(); i++)
	{
		if (Logic(i) == 0) {
			p.block(i, 0, 1, 3) = MatrixXd::Zero(1, 3);
		}
	}

	return p;
}

//Tendon Length measurement
MatrixXd AppendTendonMeasurement(const MatrixXd& Pa, const MatrixXd& Pb, const MatrixXd& L) {

	//Reshape L into a vector of series pl pr tl tr... pl pr tl tr
	MatrixXd dL = L.transpose();
	dL.resize(L.size(), 1); //Lt is now 4 by 1

	//For each tendon in Pa, Pb compute magnitude
	RowVector3d diff, A, B;

	for (int i = 0; i < Pa.rows(); i++)
	{
		//Get the vectors:
		A = Pa.block(i, 0, 1, 3); B = Pb.block(i, 0, 1, 3);
		//If they are both nonzero
		if (A.any() && B.any()) {
			//Append magnitude of tendon to total
			diff = B - A;
			dL(i) = dL(i) + diff.norm();
		}
	}	

	//Reshape and transpose to return L in original form
	dL.resize(L.cols(), L.rows());
	return dL.transpose();
}

//Tendon Length Function
MatrixXd SnakeRaven::GetTendonLengths()
{
	//Initialise Tendon Length Matrix as zeros:
	MatrixXd dLi = MatrixXd::Zero(2 * m, 2);

	//Points A and B
	MatrixXd Pa(4 * m,3); MatrixXd Pb(4 * m, 3);

	//Start Snake Kinematics from arbitrary base frame
	MatrixXd Tend = MatrixXd::Identity(4,4);

	//Start snakebot Kinematic chain
	int first_disk = 1;

	//The pan first assumption
	bool pan_first = true;

	//Loop through each module k until m
	for (int k = 0; k < m; k++)
	{
		//Decide on number of pan and tilt joints:
		double np, nt;
		if (pan_first == true) {
			np = round(n(k) / 2.0); nt = n(k) - np;
		}
		else {
			nt = round(n(k) / 2.0); np = n(k) - nt;
		}

		//Extract joint angles for pan and tilt of module k
		double Op = q(3 + 2 * k); //pan angle
		double Ot = q(4 + 2 * k); //tilt angle

								  //Get Radius of curvature from alpha:
		double r = (w / 2.0) / sin(alpha(k));

		//Compute Pan transform:
		double r01x = 0;
		double r01y = -2 * r * (1 - cos(alpha(k))*cos(Op / (2 * np)))*sin(Op / (2 * np));
		double r01z = 2 * r * (1 - cos(alpha(k))*cos(Op / (2 * np)))*cos(Op / (2 * np));
		Affine3d T01(Translation3d(r01x, r01y, r01z) * AngleAxisd((Op / np), Vector3d::UnitX()));
		Affine3d T11(Translation3d(0, 0, d(k)));
		Affine3d Tpan = T01 * T11;

		//Compute Tilt Transform:
		double r12x = 2 * r * (1 - cos(alpha(k))*cos(Ot / (2 * nt)))*sin(Ot / (2 * nt));
		double r12y = 0;
		double r12z = 2 * r * (1 - cos(alpha(k))*cos(Ot / (2 * nt)))*cos(Ot / (2 * nt));
		Affine3d T12(Translation3d(r12x, r12y, r12z) * AngleAxisd((Ot / nt), Vector3d::UnitY()));
		Affine3d T22(Translation3d(0, 0, d(k)));
		Affine3d Ttilt = T12 * T22;

		//Kinematic Chain
		for (int i = first_disk; i <= (first_disk + n(k) - 1); i++)
		{
			if (pan_first == true) {
				if (isodd(i - (first_disk - 1))) {
					//Measure Start Pa
					Pa = GetTendonPoints(Tend, w, alpha(k), d(k), 0, 1, k, m);
					Tend = Tend * Tpan.matrix(); //pan
					//Measure End Pb
					Pb = GetTendonPoints(Tend, w, alpha(k), d(k), 1, 1, k, m);
					//Append Tendon Measurement
					dLi = AppendTendonMeasurement(Pa, Pb, dLi);
				}
				else {
					//Measure Start Pa
					Pa = GetTendonPoints(Tend, w, alpha(k), d(k), 0, 0, k, m);
					Tend = Tend * Ttilt.matrix(); //tilt
					//Measure End Pb
					Pb = GetTendonPoints(Tend, w, alpha(k), d(k), 1, 0, k, m);
					//Append Tendon Measurement
					dLi = AppendTendonMeasurement(Pa, Pb, dLi);
				}
			}
			else {
				if (isodd(i - (first_disk - 1))) {
					//Measure Start Pa
					Pa = GetTendonPoints(Tend, w, alpha(k), d(k), 0, 0, k, m);
					Tend = Tend * Ttilt.matrix(); //tilt
					//Measure End Pb
					Pb = GetTendonPoints(Tend, w, alpha(k), d(k), 1, 0, k, m);
					//Append Tendon Measurement
					dLi = AppendTendonMeasurement(Pa, Pb, dLi);
				}
				else {
					//Measure Start Pa
					Pa = GetTendonPoints(Tend, w, alpha(k), d(k), 0, 1, k, m);
					Tend = Tend * Tpan.matrix(); //pan
					//Measure End Pb
					Pb = GetTendonPoints(Tend, w, alpha(k), d(k), 1, 1, k, m);
					//Append Tendon Measurement
					dLi = AppendTendonMeasurement(Pa, Pb, dLi);
				}
			}
		}

		//Transition disk:
		if (k != (m - 1)) {
			Affine3d Tpd(AngleAxisd(deg2rad(-90 / m), Vector3d::UnitZ()));
			Tend = Tend * Tpd.matrix();
		}
		//Next section decide if it starts as a pan or tilt
		if ((pan_first == true) && (isodd(n(k)))) {
			pan_first = false;
		}
		else if ((pan_first == false) && (isodd(n(k)))) {
			pan_first = true;
		}
		else if ((pan_first == true) && (isodd(n(k)) == false)) {
			pan_first = true;
		}
		else if ((pan_first == false) && (isodd(n(k)) == false)) {
			pan_first = false;
		}
		//Update first disk for the next segment:
		first_disk = first_disk + static_cast<int>(n(k));
	}
	//Return total tendon length calculation
	return dLi;
}

//Motor Values Function
VectorXd SnakeRaven::q2Motor_angles()
{
	//Set the motor angles as the last q
	mv = q;

	//Find Change in Tendon Lengths
	dLi = GetTendonLengths(); //Current lengths
	ddL = dLi - dL0; //subtract from neutral

	//Extract the tendons under tension
	//
	for (int i = 0; i < (2 * m); i++)
	{
		if (ddL(i, 0) <= ddL(i, 1)) {
			//if left tendon is smaller than Right tendon
			//Positive rotation
			mv(3 + i) = ddL(i, 0) / (da / 2.0);
		}
		else if (ddL(i, 0) > ddL(i, 1)) {
			//if left tendon is longer than Right tendon
			//Negative rotation
			mv(3 + i) = -ddL(i, 1) / (da / 2.0);
		}
	}

	//Go through calibration values:
	for (int i = 0; i < DOF; i++)
	{
		mv(i) = rate(i)*mv(i) + offset(i);
	}

	//Return motor position
	return mv;
}





//*****************KINEMATICS MATHS FUNCTIONS*********************//

MatrixXd dh(const double alpha, const double a, const double d, const double theta)
{
	//DH parameter matrix construction
	const double c0 = cos(theta);
	const double s0 = sin(theta);
	const double ca = cos(alpha);
	const double sa = sin(alpha);

	Matrix4d T;
	T << c0, -s0, 0, a,
		s0*ca, c0*ca, -sa, -d*sa,
		s0*sa, c0*sa, ca, d*ca,
		0, 0, 0, 1;
	return T;
}

bool isodd(double v) {
	//if there is a remainder after integer division by 2
	if (static_cast<int>(v) % 2) 
		return true;
	else
		return false;
}

double deg2rad(double degrees) {
	//convert degrees to radians
	return degrees * PI / 180;
}

double rad2deg(double radians) {
	//convert radians to degrees
	return radians * 180 / PI;
}

MatrixXd skew(const VectorXd& v)
{
	//Turn a 3x1 vector into a skew symmetric matrix
	Matrix3d S;
	S << 0, -v(2), v(1),
		v(2), 0, -v(0),
		-v(1), v(0), 0;
	return S;
}

VectorXd vex(const MatrixXd& S)
{
	//Turn a skew symmetric matrix into a 3x1 vector
	Vector3d v;
	v << S(2,1), S(0,2), S(1,0);
	return v;
}

MatrixXd trans2dx(const MatrixXd& T0, const MatrixXd& T1)
{
	//Turn a difference in transformation matrices into a delta vector
	Vector3d t1, t0;
	Matrix3d R0, R1;
	//Extract position:
	t0 = T0.block(0, 3, 3, 1);
	t1 = T1.block(0, 3, 3, 1);
	//Extract rotation:
	R0 = T0.block(0, 0, 3, 3);
	R1 = T1.block(0, 0, 3, 3);
	//Create 6DOF delta vector
	Matrix<double, 6, 1> dx;
	dx.block(0, 0, 3, 1) = t1 - t0;
	dx.block(3, 0, 3, 1) = vex(R1 * R0.transpose() - Matrix3d::Identity(3, 3));
	return dx;
}

VectorXd cap_mag(const VectorXd& V, const double magnitude)
{
	//saturates the magnitude of a vector to be a certain magnitude
	return magnitude * (V / V.norm());
}

MatrixXd applyJointLimits(const MatrixXd& q, const MatrixXd& ql, const MatrixXd& qu)
{
	//Saturates joint vector q to be within ql and qu
	MatrixXd q_ = q;

	//For each degree of freedom check the joint limits ql qu
	int DOF = q.size();
	for (int i = 0; i < DOF; i++)
	{
		if (q(i) < ql(i)) {
			q_(i) = ql(i);
		}
		else if (q(i) > qu(i)) {
			q_(i) = qu(i);
		}
	}

	//Return q but with saturation
	return q_;
}


MatrixXd dampedLeastSquares(const MatrixXd& J, const MatrixXd& q, const MatrixXd& ql, const MatrixXd& qu)
{
	//Computes a pseudo inverse of J minimising q and the distance to limits ql and qu

	//Damping Matrix:
	int DOF = q.size();
	MatrixXd D = MatrixXd::Identity(DOF, DOF);
	//Penalty gains
	double c, p, w; 
	c = 1; p = 2; w = 1;
	
	//Generate peanalty terms in damping matrix:
	double num, den;
	for (int i = 0; i < DOF; i++)
	{
		num = 2 * q(i) - qu(i) - ql(i);
		den = qu(i) - ql(i);
		D(i, i) = c*pow((num / den),p) + (1 / w);
	}

	//Now compute the Pseudo Inverse of the Jacobian
	MatrixXd inv_J;
	inv_J = (J.transpose() * J + D * D).inverse() * J.transpose();

	//Matrix<double, 3 + 2 * Max_m, 6> inv_J;
	return inv_J;
}


MatrixXd TransformPoints(const MatrixXd& T, const MatrixXd& p)
{
	//Compute points p in reference transform T
	// P = T*p where p is a matrix with columns [x y z]
	int n = p.rows();
	MatrixXd P = MatrixXd::Zero(n,3);
	Vector4d ph, Pv;

	for (int i = 0; i < n; i++)
	{
		ph << p(i, 0), p(i, 1), p(i, 2), 1;
		Pv = T * ph;
		P(i, 0) = Pv(0);
		P(i, 1) = Pv(1);
		P(i, 2) = Pv(2);
	}
	return P;
}