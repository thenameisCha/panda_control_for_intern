#pragma once

// Copyright (c) 2018-2022 Seoul National University - Suhan Park (psh117@snu.ac.kr) @ DYROS 
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <iostream>
#include <Eigen/Dense>
#include <rbdl/rbdl.h>
#include <memory>
#include <fstream>
#include "math_type_define.h"


// TODO: to exp with real robot, add
#include "franka_model_interface.h"
// ---


#define EYE(X) Matrix<double, X, X>::Identity()

using namespace RigidBodyDynamics;
using namespace std;
using namespace Eigen;

class ArmController
{
	size_t dof_;

	// Initial state
		//Joint space
	Vector7d q_init_;
	Vector7d q_dot_init_;
		//Cartesian space
	Vector3d x_init_linear;
	Vector6d x_init_6d;
	Vector12d x_init_12d;
	Matrix3d rotation_init_;
	Vector3d rotation_init_in_vector;
	Vector6d x_dot_init_;
	// Current state
		//Joint space
	Vector7d q_;
	Vector7d q_dot_;
	Vector7d q_ddot_;
	Vector7d torque_;
		//Cartesian space
	Vector12d x_12d_;
	Vector6d x_6d_;
	Vector3d x_linear_;
	Matrix3d rotation_;
	Vector3d rotation_in_vector;
	Vector6d x_dot_; // 6D (linear + angular)
	Vector6d F_ext_new; // current F_ext
	Vector6d F_ext_old; // previous F_ext

	// Target state
		//Joint space
	Vector7d q_target;
	Vector7d q_dot_target;
		//Cartesian space
	Vector12d x_target_12d; // p and r1 r2 r3
	Vector6d x_target_6d;
	Vector6d x_dot_target;
	Matrix3d rotation_target;
	Vector3d rotation_target_vector;

	// Desired state(cubic splined)
		//Joint space
	Vector7d q_desired_;
	Vector7d q_desired_dot_;
		//Cartesian space
	Vector3d x_desired_linear_; //linear part of x_desired
	Vector3d x_desired_angular; //angular part of x_desired
	Vector12d x_desired_12d;
	Vector6d x_desired_6d;
	Vector6d x_desired_6d_calculated;
	Vector6d x_desired_dot_;
	Vector6d x_desired_dot_f;
	Matrix3d rotation_desired_;
	Vector6d F_desired;
	
	// Error value
		//Joint space
	Vector7d q_error_; //q_desired - q_
	Vector7d q_error_dot_; //q_desired_dot_ - q_dot_
	Vector7d q_error_integral;
		//Cartesian space
	Vector6d x_error_; //x_desired - x_
	Vector6d x_error_integral;
	Vector6d x_error_dot_; //x_desired_dot - x_dot_
	Vector3d phi_; //Rotation error. Must be included into x_error

	// Control value : Control input torque tau = tau_imp + tau_force
		//Joint space
	Vector7d position_input;
		//Cartesian space
	Vector7d tau_imp;
	Vector7d tau_force;
	Vector7d torque_input;
		//Calculated using these values
	Vector6d F_F;
	Vector6d F_integral;
	Vector6d F_ext_dot;	
	Vector6d x_error_dot_control;
	Vector6d x_dot_control;
	Vector6d x_desired_dot_control;

	// For Tank
	Vector3d port_power; //[<xdot,F_F>, <x_D_dot, -F_F>, <X_D_dot, -F_ext>]
	Vector3d port_gamma;
	double S_ur;
	double tank_E;
	double tank_power; // power before gamma manipulation
	double tank_state;

	// Dynamics
	Vector7d g_; // Gravity torque
	Matrix7d m_; // Mass matrix
	Matrix7d m_inverse_; // Inverse of mass matrix
	Matrix6d lambda_;

	// For controller
	Matrix<double, 3, 7> j_v_;	// Linear velocity Jacobian matrix
	Matrix<double, 3, 7> j_w_;	// Angular veolicty Jacobain matrix
	Matrix<double, 6, 7> j_;	// Full basic Jacobian matrix
	Matrix<double, 7, 6> j_inverse_;	// Jacobain inverse storage 

	VectorXd q_temp_;	// For RBDL 
	VectorXd q_dot_temp_;
	VectorXd q_ddot_temp_;
	MatrixXd j_temp_;	// For RBDL 
	MatrixXd m_temp_;
	VectorXd g_temp_;   // For RBDL 

	unsigned long tick_;
	double play_time_;
	double hz_;
	double control_start_time_;
	Matrix6d external_wrench_error;

	std::string control_mode_;
	bool is_mode_changed_;

	Matrix6d Kx, Dx,Ix, Kp, Kd, Ki; //ETank mode parameters
	Matrix7d kp, kv, ki, joint_damping;	//Home mode parameters

	// for robot model construction
	Math::Vector3d com_position_[DOF];
	Vector3d joint_posision_[DOF];

	shared_ptr<Model> model_;
	unsigned int body_id_[DOF];
	Body body_[DOF];
	Joint joint_[DOF];
	
	// TODO: to exp with real robot, add
	FrankaModelInterface model_interface_;
	// ---
private:
	void printState();
	void moveJointPositionTorque(double duration);
	void ETank(double duration);
	void Test();
	void initialize_ETank_controller();
	void initialize_Home_controller();
	void initialize_Force_Test();
	void calculate_desired_ETank(double duration);
	void calculate_desired_Home(double duration);
	void calculate_desired_Test();
	void update_port_power();
	void update_gamma();
	void update_tank();
	void update_torque_input();
	void indi_power_limit();
	void tank_power_limit();
	void tank_energy_limit();
	void EGS();
	void WGS();
	void SGA();

public:
	void readData(const Vector7d &position, const Vector7d &velocity, const Vector7d &coriolis);
	void readData(const Vector7d &position, const Vector7d &velocity, const Vector7d &coriolis, const Vector6d &ee_force);	void readData(const Vector7d &position, const Vector7d &velocity);
	void readGravityData(const Vector7d &gravity);
	const Vector7d & getPositionInput();
	const Vector7d & getTorqueInput(); // return tau_input
	const Vector6d & getDesiredForce(); // return F_desired	

public:
		ArmController(double hz, franka::Model &model) :
		tick_(0), play_time_(0.0), hz_(hz), control_mode_("none"), is_mode_changed_(false), model_interface_(model)
	{
			initDimension(); initModel(); initFile();
	}


    void setMode(const std::string & mode);
    void setIsChanged(bool is_changed);
	void initDimension();
    void initModel();
	void initFile();
    void initPosition(franka::RobotState state);
    void compute();
	void move(); // depending on mode, calculate target, desired position, torque input.
	void mode_is_changed(); // when mode is changed, set init_ parameters and target parameters
	void readCurrentTime(double time) { play_time_ = time; };
private:
	ofstream debug_file_;
	constexpr static int NUM_HW_PLOT{1};
	ofstream hw_plot_files_[NUM_HW_PLOT];
	const string hw_plot_file_names_[NUM_HW_PLOT]
	{"ETank_data"};					// 10 11

	void record(int file_number, double duration);
	void record(double duration);
	void record(int file_number, double duration, const stringstream & ss);
};
