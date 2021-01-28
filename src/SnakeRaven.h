#ifndef SNAKERAVEN_H_
#define SNAKERAVEN_H_
/*
SnakeRaven Class and kinematics maths functions declarations

author: Andrew Razjigaev 2020
*/

//Set the maximum amount of modules for Snake Raven to have
//This is needed for memory allocation as object's internal size cannot grow
//You also cannot set it to 2 and use 1 it must be set to the current design model

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

using namespace Eigen;
using namespace std;

#define PI           3.14159265358979323846  /* pi */

//Set the maximum amount of modules for Snake Raven to have
//This is needed for memory allocation as object's internal size cannot grow
//You also cannot set it to 2 and use 1 it must be set to the current design model
#define Max_m 2
#define history 10

/*
Kinematics Math Functions for the Snake Raven Manipulator

It defines constants PI and the maximum number of modules

It includes these functions with support from Eigen:
- DH parameter matrix function: dh
- is odd logic function: isodd
- degrees to radians function: deg2rad
- Skew symmetric matrix function: skew
- Inverse skew (vex) function: vex
- transformation to delta vector function: trans2dx
- cap the magnitude of a vector function: cap_mag
- Compute the damped least squares pseudo inverse of a Jacobian: dampedLeastSquares
- Restrict a joint vector to be within its limits: applyJointLimits
- Transform Points to a coordinate frame with: TransformPoints

author: Andrew Razjigaev 2019
*/

//Kinematics functions:
MatrixXd dh(const double alpha, const double a, const double d, const double theta);

bool isodd(double v);

double deg2rad(double degrees);

double rad2deg(double radians);

MatrixXd skew(const VectorXd& v);

VectorXd vex(const MatrixXd& S);

MatrixXd trans2dx(const MatrixXd& T0, const MatrixXd& T1);

VectorXd cap_mag(const VectorXd& V, const double magnitude);

MatrixXd applyJointLimits(const MatrixXd& q, const MatrixXd& ql, const MatrixXd& qu);

bool JointLimitcheck(const MatrixXd& q, const MatrixXd& ql, const MatrixXd& qu);

MatrixXd dampedLeastSquares(const MatrixXd& J, const MatrixXd& q, const MatrixXd& ql, const MatrixXd& qu);

MatrixXd TransformPoints(const MatrixXd& T, const MatrixXd& p);

/*
SnakeRaven Class

This file defined the class SnakeRaven and its functions:
- Constructor with default properties and gene paramters
- Forward Kinematics for a generic SnakeRaven
- Jacobian matrix for a generic SnakeRaven
- Get Tendon Lengths for a for a generic SnakeRaven using
 - GetTendonPoints
 - AppendTendonMeasurements
- q2Motor_angles conversion function that computes rotation of the Raven motors


author: Andrew Razjigaev 2019
*/

//Snake Raven Class
class SnakeRaven
{
public:
	//Initialise:
	//The parameter Matrix
	Matrix<double, Max_m, 1>  alpha, n, d;
	double w, da, ra; //w is width of tube, da is actuation diameter, ra is its radius

	//Number of modules and Degrees of freedom of this object:
	int m, DOF;

	//Endeffector Pose 4 by 4 matrix
	Matrix4d Tend;

	//Endeffector and Tool Pose 4 by 4 Transform
	Affine3d Tool;

	//Is right or left Raven arm:
	bool isrightarm;

	//Joint vector and joint limits, motor values, calibration
	Matrix<double, 3 + 2 * Max_m, 1> q, ql, qu, mv, mv_pre, mv_pre_ave, rate, offset, q_temp, q_desired, mv_coupling;

	//motor value history
	Matrix<double, 3 + 2 * Max_m, history> mv_history;

	//Jacobian Matrix: a 6 by DOF matrix
	Matrix<double, 6, 3 + 2 * Max_m> J;

	//Initialise Tendon Matrices:
	Matrix<double, 2 * Max_m, 2> dL0, dLi, ddL;

	//Control Weighting Matrix for Damped Least Squares
	DiagonalMatrix<double, 6> W;

	//Construct class with given default properties:
	SnakeRaven();

	//Initialise SnakeRaven Functions:
	void reset_q(); //makes q = initial state
	MatrixXd get_mv_pre_ave();
	MatrixXd FK(); //Forward Kinematics
	MatrixXd Jacobian(); //Jacobian matrix
	MatrixXd GetTendonLengths(); //Tendon Length function
	VectorXd q2Motor_angles(); //convert q to motor angles on Raven II
	VectorXd Motor_angles2q(); //convert motor angles from Raven II to q

};

MatrixXd GetTendonPoints(const MatrixXd& T, const double w, const double alpha, const double d, bool B, bool pan, const int k, const int M);
MatrixXd AppendTendonMeasurement(const MatrixXd& Pa, const MatrixXd& Pb, const MatrixXd& L);

#endif