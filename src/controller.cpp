
// Copyright (c) 2018-2022 Seoul National University - Suhan Park (psh117@snu.ac.kr) @ DYROS 
// Use of this source code is governed by the Apache-2.0 license, see LICENSE


#include "controller.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#define TANK_INITIAL_E 12 //Set initial tank energy level as 12J
#define TANK_E_UPPER_LIMIT 13
#define TANK_E_LOWER_LIMIT 10
#define TANK_E_SMOOTHING 0.5
#define TANK_E_EPSILON 0.02
#define PORT_POWER_UPPER_LIMIT_2 0.6
#define PORT_POWER_LOWER_LIMIT_2 -0.6
#define PORT_POWER_UPPER_LIMIT_1 0.2
#define PORT_POWER_LOWER_LIMIT_1 -0.2
#define PORT_POWER_UPPER_LIMIT_0 0.2
#define PORT_POWER_LOWER_LIMIT_0 -0.2
#define TANK_POWER_UPPER_LIMIT 0.7
#define TANK_POWER_LOWER_LIMIT -0.7
#define S_UR_MAX 0.1
#define S_UR_SMOOTHING 0.02
#define DESIRED_FORCE 10



void ArmController::compute()
{
	// Kinematics and dynamics calculation ------------------------------
	Eigen::Affine3d transform = model_interface_.getTransform(franka::Frame::kEndEffector, q_);
	x_linear_ = transform.translation();
	rotation_ = transform.linear();
	j_ = model_interface_.getJacobianMatrix(franka::Frame::kEndEffector, q_);
	m_ = model_interface_.getMassMatrix(q_);
	m_inverse_ = m_.inverse();
	j_v_ = j_.block < 3, DOF>(0, 0);
	lambda_ = (j_ * m_inverse_ * j_.transpose()).inverse();
	j_inverse_ = m_inverse_ * j_.transpose() * lambda_; // Advanced robot manipulation pg 84
	
	//calculate current values
		//task space
			//Position
				//12d
	x_12d_.segment<3>(0) = x_linear_;
	for (int i = 0; i < 3; i++){
		x_12d_.segment<3>(3 + i * 3) = rotation_.block<3, 1>(0, i);
	}
				//6d
	Matrix3d rot_log = rotation_.log();
	rotation_in_vector << rot_log(2,1), rot_log(0,2), rot_log(1,0);
	x_6d_ << x_linear_, rotation_in_vector;
			//Velocity
				//6d
	x_dot_ = j_ * q_dot_;
			//Acceleration
	x_ddot_ = j_ * q_ddot_;

	//Modify F_Ext
	F_ext_new += lambda_ * x_ddot_;
	
	if (is_mode_changed_) mode_is_changed();
	move();
	printState();
	record(20.0);

	tick_++;
	//play_time_ = tick_ / hz_;	// second
}

void ArmController::mode_is_changed(){
	//calculate new start time, init, and target
	if (control_mode_ == "home"){
		initialize_Home_controller();
	}
	else if (control_mode_ == "ETank"){
		initialize_ETank_controller();
	}
	else if (control_mode_ == "Test"){
		initialize_Force_Test();
	}
}

void ArmController::move(){
	if (control_mode_ == "home") moveJointPositionTorque(3.0);
	else if (control_mode_ == "ETank") ETank(1.0);
	else if (control_mode_ == "Test") Test();
}

void ArmController::moveJointPositionTorque(double duration){
	calculate_desired_Home(duration);
	update_torque_input();
}

void ArmController::ETank(double duration){
	port_gamma << 1, 1, 1; //initialize valve gain
	calculate_desired_ETank(duration);
	update_port_power();
	update_gamma();	
	update_tank();
	update_torque_input();
}

void ArmController::Test(){
	port_gamma << 1, 1, 1; //initialize valve gain
	calculate_desired_Test();
	update_port_power();
	//update_gamma();	
	update_tank();
	update_torque_input();
	record(1.0);
}

void ArmController::initialize_Home_controller(){
	kp = Matrix7d::Identity() * 50.0;
	kv = Matrix7d::Identity() * 2;

	is_mode_changed_ = false;
	control_start_time_ = play_time_;
	q_init_ = q_;
	q_dot_init_ = q_dot_;
	q_target << 0, 0, 0, -M_PI/2, 0, M_PI/2, 0;
}

void ArmController::initialize_ETank_controller(){
	F_desired.setZero();
	F_ext_new.setZero();
	F_ext_old.setZero();
	F_integral.setZero();
	x_error_integral.setZero();
	q_error_integral.setZero();
	tank_E = TANK_INITIAL_E;
	float scale = 1.0; // for impedance control
	float scale2 = 0.01; // for force control
	float scale3 = 0.35; // for joint space damping
	Kx = scale * Matrix6d::Identity(); //orientation 에 해당하는 gain must be small ~ 1.0. linear gain ~100
	Dx = scale * Matrix6d::Identity();
	Ix = 2.3 * Matrix6d::Identity();
	Kp =  scale2 * Matrix6d::Identity();
	Kp.topLeftCorner(3,3) = Kp.topLeftCorner(3,3) * 50;
	Kp.bottomRightCorner(3,3) = Kp.bottomRightCorner(3,3) * 20;
	Kd = 0*scale2 * Matrix6d::Identity();
	Ki = 0 * scale2 * Matrix6d::Identity();
	Ki.topLeftCorner(3,3) = Ki.topLeftCorner(3,3) * 10;
	Ki.bottomRightCorner(3,3) = Ki.bottomRightCorner(3,3) * 1;

	Kx.topLeftCorner(3,3) = Kx.topLeftCorner(3,3) * 1000; //450
	Dx.topLeftCorner(3,3) = Dx.topLeftCorner(3,3) * 80;  //40
	Ix.topLeftCorner(3,3) = Ix.topLeftCorner(3,3) * 110;  //110
	Kx.bottomRightCorner(3,3) = Kx.bottomRightCorner(3,3) * 100;
	Dx.bottomRightCorner(3,3) = Dx.bottomRightCorner(3,3) * 10;
	joint_damping = scale3 * Matrix7d::Identity();
	F_ext_Error.setZero();
	
	is_mode_changed_ = false;
	is_wrench_modified = 0;
	control_start_time_ = play_time_;
	x_init_12d = x_12d_;
	x_init_6d = x_6d_;
	x_init_linear = x_linear_;
	x_dot_init_ = x_dot_;
	rotation_init_ = rotation_;
	rotation_init_in_vector = rotation_in_vector;
	x_target_12d = x_init_12d;
	x_target_12d(2) -= 0.2;
	x_target_6d = x_init_6d;
	x_target_6d(2) -= 0.2;
	rotation_target = rotation_init_;
	rotation_target_vector = rotation_init_in_vector;
	x_desired_dot_f << 0.0, 0.0, -0.04, 0, 0, 0; // FOR TEST!!!!
}

void ArmController::initialize_Force_Test(){
	F_desired.setZero();
	F_ext_new.setZero();
	F_ext_old.setZero();
	F_integral.setZero();
	x_error_integral.setZero();
	q_error_integral.setZero();
	tank_E = TANK_INITIAL_E;
	float scale = 1.0; // for impedance control
	float scale2 = 0.1; // for force control
	float scale3 = 0.35; // for joint space damping
	Kx = scale * Matrix6d::Identity(); //orientation 에 해당하는 gain must be small ~ 1.0. linear gain ~100
	Dx = scale * Matrix6d::Identity();
	Ix = 2.3 * Matrix6d::Identity();
	Kp = 1 * scale2 * Matrix6d::Identity();
	Kd = 0*scale2 * Matrix6d::Identity();
	Ki = 0 * scale2 * Matrix6d::Identity();
	Kx.topLeftCorner(3,3) = Kx.topLeftCorner(3,3) * 30000; //450
	Dx.topLeftCorner(3,3) = Dx.topLeftCorner(3,3) * 0;  //40
	Ix.topLeftCorner(3,3) = Ix.topLeftCorner(3,3) * 0;  //110
	Kx.bottomRightCorner(3,3) = Kx.bottomRightCorner(3,3) * 10;
	Dx.bottomRightCorner(3,3) = Dx.bottomRightCorner(3,3) * 0;
	joint_damping = scale3 * Matrix7d::Identity();
	F_ext_Error.setZero();
	
	is_mode_changed_ = false;
	is_wrench_modified = 0;
	control_start_time_ = play_time_;
	x_init_12d = x_12d_;
	x_init_6d = x_6d_;
	x_init_linear = x_linear_;
	x_dot_init_ = x_dot_;
	rotation_init_ = rotation_;
	rotation_init_in_vector = rotation_in_vector;
	x_target_12d = x_init_12d;
	x_target_6d = x_init_6d;
	rotation_target = rotation_init_;
	rotation_target_vector = rotation_init_in_vector;
}

void ArmController::calculate_desired_Home(double duration){
	for (int i = 0; i < 7; i++){
		q_desired_dot_(i) = DyrosMath::cubicDot(play_time_, control_start_time_,
			control_start_time_ + duration, q_init_(i), q_target(i), q_dot_init_(i), 0);
		q_desired_(i) = DyrosMath::cubic(play_time_, control_start_time_,
			control_start_time_ + duration, q_init_(i), q_target(i), q_dot_init_(i), 0);
	}
}

void ArmController::calculate_desired_ETank(double duration){
//x_desired calculated
	if (play_time_ == control_start_time_) {
		x_desired_linear_ = x_init_linear;
		rotation_desired_ = rotation_init_;
	}
	else {
		x_desired_linear_ += x_desired_dot_.segment<3>(0) / hz_; // xdesired(t+dt) = xdesired(t) + dt * xdesired_dot(t)
	}
	x_desired_12d << x_desired_linear_, rotation_desired_.block<3,1>(0,0), rotation_desired_.block<3,1>(0,1), rotation_desired_.block<3,1>(0,2);

	//Calculate Error
	Vector3d x_error_linear = x_desired_linear_ - x_linear_;
	Vector3d x_error_angular = -DyrosMath::getPhi(rotation_, rotation_desired_);
	x_error_ << x_error_linear, x_error_angular;
	// x_error_ = x_desired_6d - x_6d_;
	x_error_dot_ = x_desired_dot_ - x_dot_;
	x_error_integral += x_error_ / hz_;
	x_desired_dot_ = x_desired_dot_f;
}

void ArmController::calculate_desired_Test(){
	//x_desired calculated
	x_desired_dot_.setZero();
	x_desired_linear_ = x_init_6d.segment<3>(0);
	x_desired_angular = x_init_6d.segment<3>(3);
	Eigen::Matrix3d skew;
	skew << 0, -x_desired_angular[5], x_desired_angular[4],
						x_desired_angular[5], 0, -x_desired_angular[3],
						-x_desired_angular[4], x_desired_angular[3], 0;
	rotation_desired_ = skew.exp();
	x_desired_12d << x_desired_linear_, rotation_desired_.block<3,1>(0,0), rotation_desired_.block<3,1>(0,1), rotation_desired_.block<3,1>(0,2);
	x_desired_6d << x_desired_linear_, x_desired_angular;
	x_error_ = x_desired_6d - x_6d_;
	x_error_dot_ = x_desired_dot_ - x_dot_;
	x_error_integral += x_error_ / hz_;
}

void ArmController::update_port_power(){
//Calculate Port Individual Power
	port_power[0] = x_dot_.dot(-F_F);
	port_power[1] = x_desired_dot_.dot(F_F); 
	port_power[2] = x_desired_dot_.dot(F_ext_new);
}

void ArmController::update_gamma(){
	//policy applied
	indi_power_limit();
	tank_power_limit();
	tank_energy_limit();
	// x_desired_dot_control = (port_gamma[1]+port_gamma[2])/2 * x_desired_dot_; //average mode
	// x_desired_dot_control = min(port_gamma[1], port_gamma[2]) * x_desired_dot_; //conservative mode
	x_desired_dot_control = port_gamma[1] *port_gamma[2] * x_desired_dot_; //multiply mode
	x_dot_control = port_gamma[0] * x_dot_;
	x_desired_dot_ = x_desired_dot_control;
	x_error_dot_control = x_desired_dot_control- x_dot_control;
}

void ArmController::update_tank(){
	//Calculate Tank State
	tank_power = port_power[0] + port_power[1] + port_power[2]; // -> port_power[1] < 0 when f_ext too big, port_power[2] < 0, port_power[0] = 0
	tank_state = sqrt(2*tank_E);
	float state_dot = 1/tank_state * port_gamma.dot(port_power);
	tank_state += state_dot/hz_;
	tank_E = 0.5 * pow(tank_state,2);
	S_ur = 0.5 * x_error_.transpose() * Kx * x_error_ + 0.5 * ((x_desired_dot_ - x_dot_).transpose() * lambda_).dot((x_desired_dot_ - x_dot_));
}

void ArmController::update_torque_input(){
//Calculate torque input
	if (control_mode_ == "home"){
	torque_input = m_ * (kp*(q_desired_ - q_) + kv*(q_desired_dot_ - q_dot_)) + g_; 
	}
	else if (control_mode_ == "ETank"){
	//Calculate impedance control input
	tau_imp = j_.transpose() * (Kx*x_error_ + Dx*(-x_dot_control));//+ Ix*x_error_integral); // This method's gamma does not affect x_error or force control
	
	//Calculate F_F
	if (tick_ % 1000 == 0) F_integral.setZero();
	F_integral += (F_ext_new-F_desired)/hz_;
	F_F =  Kp*(F_ext_new-F_desired) + Ki*(F_integral);
	tau_force = j_.transpose() * F_F;
	torque_input = g_+ tau_imp + tau_force;// - joint_damping * q_dot_;
	}
	else if (control_mode_ == "Test"){
		x_error_dot_control = x_desired_dot_control- x_dot_control;
		tau_imp = j_.transpose() * (Kx*x_error_ + Dx*(-x_error_dot_));
		torque_input = g_+ tau_imp;// - joint_damping * q_dot_;
	}
}

void ArmController::indi_power_limit(){
	
		// individual port power limit
	float pt_low, pt_upper;
	if (S_ur <= S_UR_MAX - S_UR_SMOOTHING) {
		pt_low = PORT_POWER_LOWER_LIMIT_2;
		pt_upper = PORT_POWER_UPPER_LIMIT_2;
	}
	else if(S_ur <= S_UR_MAX && S_ur >= S_UR_MAX - S_UR_SMOOTHING){
		pt_low = 0.5 * PORT_POWER_LOWER_LIMIT_2 * (1-cos((S_UR_MAX - S_ur) * M_PI / S_UR_SMOOTHING));
		pt_upper = 0.5 * PORT_POWER_UPPER_LIMIT_2 * (1-cos((S_UR_MAX - S_ur) * M_PI / S_UR_SMOOTHING));
	}
	else {
		pt_low = 0;
		pt_upper = 0;
	}
	// FIRST METHOD
	// if (port_gamma[2] * port_power[2] > PORT_POWER_UPPER_LIMIT_2 && PORT_POWER_UPPER_LIMIT_2 >= 0){
	// 	port_gamma[2] =  pt_upper / port_power[2];
	// }
	// else if (port_gamma[2] * port_power[2] < PORT_POWER_LOWER_LIMIT_2 && PORT_POWER_LOWER_LIMIT_2 <= 0){
	// 	port_gamma[2] = pt_low / port_power[2];
	// }
	// if (port_gamma[1] * port_power[1] > PORT_POWER_UPPER_LIMIT_1 && PORT_POWER_UPPER_LIMIT_1 >= 0){
	// 	port_gamma[1] = pt_upper / port_power[1];
	// }
	// else if (port_gamma[1] * port_power[1] < PORT_POWER_LOWER_LIMIT_1 && PORT_POWER_LOWER_LIMIT_1 <= 0){
	// 	port_gamma[1] =pt_low / port_power[1];
	// }
	// if (port_gamma[0] * port_power[0] > PORT_POWER_UPPER_LIMIT_0 && PORT_POWER_UPPER_LIMIT_0 >= 0){
	// 	port_gamma[0] = pt_upper / port_power[0];
	// }
	// else if (port_gamma[0] * port_power[0] < PORT_POWER_LOWER_LIMIT_0 && PORT_POWER_LOWER_LIMIT_0 <= 0){
	// 	port_gamma[0] =  pt_low / port_power[0];
	// }
	
	// SECOND METHOD
	if (port_gamma[2] * port_power[2] > PORT_POWER_UPPER_LIMIT_2 && PORT_POWER_UPPER_LIMIT_2 >= 0){
		port_gamma[2] =  0;
	}
	else if (port_gamma[2] * port_power[2] < PORT_POWER_LOWER_LIMIT_2 && PORT_POWER_LOWER_LIMIT_2 <= 0){
		port_gamma[2] =0;
	}
	if (port_gamma[1] * port_power[1] > PORT_POWER_UPPER_LIMIT_1 && PORT_POWER_UPPER_LIMIT_1 >= 0){
		port_gamma[1] = 0;
	}
	else if (port_gamma[1] * port_power[1] < PORT_POWER_LOWER_LIMIT_1 && PORT_POWER_LOWER_LIMIT_1 <= 0){
		port_gamma[1] =0;
	}
	if (port_gamma[0] * port_power[0] > PORT_POWER_UPPER_LIMIT_0 && PORT_POWER_UPPER_LIMIT_0 >= 0){
		port_gamma[0] = 0;
	}
	else if (port_gamma[0] * port_power[0] < PORT_POWER_LOWER_LIMIT_0 && PORT_POWER_LOWER_LIMIT_0 <= 0){
		port_gamma[0] =  0;
	}
}

void ArmController::tank_power_limit(){
// Choose between the two methods
	//EGS();
	SGA();
}

void ArmController::EGS(){
//EGS
	if (port_gamma.dot(port_power) > TANK_POWER_UPPER_LIMIT && TANK_POWER_UPPER_LIMIT >= 0){
		for (int i = 0; i < 3; i++) port_gamma[i] = port_gamma[0] * TANK_POWER_UPPER_LIMIT / (port_gamma.dot(port_power));
	}
	else if (port_gamma.dot(port_power) < TANK_POWER_LOWER_LIMIT && TANK_POWER_LOWER_LIMIT <= 0){
		for (int i = 0; i < 3; i++) port_gamma[i] = port_gamma[0] * TANK_POWER_LOWER_LIMIT / (port_gamma.dot(port_power));
	}
}
void ArmController::SGA(){
	int a, b, c; 
	a = 2; b = 1; c = 0; // priorities
	if ((port_gamma[a] * port_power[a] > TANK_POWER_UPPER_LIMIT && TANK_POWER_UPPER_LIMIT >= 0) ||
		(port_gamma[a] * port_power[a] < TANK_POWER_LOWER_LIMIT && TANK_POWER_LOWER_LIMIT <= 0)) port_gamma[a] = 0;
	if ((port_gamma[a] * port_power[a] + port_gamma[b] * port_power[b] > TANK_POWER_UPPER_LIMIT 
											&& TANK_POWER_UPPER_LIMIT >= 0) ||
		(port_gamma[a] * port_power[a] + port_gamma[b] * port_power[b] < TANK_POWER_LOWER_LIMIT 
											&& TANK_POWER_LOWER_LIMIT <= 0)) port_gamma[b] = 0;
	if ((port_gamma.dot(port_power) > TANK_POWER_UPPER_LIMIT 
											&& TANK_POWER_UPPER_LIMIT >= 0) ||
		(port_gamma.dot(port_power) < TANK_POWER_LOWER_LIMIT 
											&& TANK_POWER_LOWER_LIMIT <= 0)) port_gamma[c] = 0;
}

void ArmController::tank_energy_limit(){
	for (int i = 0; i < 3; i++){
		if (tank_E >= TANK_E_UPPER_LIMIT && port_power[i] >= 0) port_gamma[i] = 0;
		else if (tank_E >= TANK_E_UPPER_LIMIT - TANK_E_SMOOTHING && tank_E <= TANK_E_UPPER_LIMIT && port_power[i] >= -TANK_E_EPSILON)
			port_gamma[i] = 0.5 * (1-cos((TANK_E_UPPER_LIMIT - tank_E) * M_PI /  TANK_E_SMOOTHING)) * port_gamma[i];
		if (tank_E <= TANK_E_LOWER_LIMIT && port_power[i] <= 0) port_gamma[i] = 0;
		else if (tank_E <= TANK_E_LOWER_LIMIT + TANK_E_SMOOTHING && tank_E >= TANK_E_LOWER_LIMIT && port_power[i] <= TANK_E_EPSILON)
			port_gamma[i] = 0.5 * (1-cos((-TANK_E_LOWER_LIMIT + tank_E) * M_PI / TANK_E_SMOOTHING)) * port_gamma[i];
	}
}


// Controller Core Methods ----------------------------

void ArmController::setMode(const std::string & mode)
{
	
	if (control_mode_ != mode) {
		control_mode_ = mode;
		is_mode_changed_ = true;
		cout << "Is already " << control_mode_ << " mode" << endl;
	}
	cout << "Current mode : " << mode << endl;
}
void ArmController::initDimension()
{
	dof_ = DOF;
	q_temp_.resize(DOF);
	j_temp_.resize(6, DOF);
	g_temp_.resize(DOF);
	m_temp_.resize(DOF, DOF);
	q_ddot_.setZero();

	x_target_6d.setZero();
	x_target_12d.setZero();
	q_desired_.setZero();
	q_desired_dot_.setZero();
	q_target.setZero();
	torque_input.setZero();

}

void ArmController::initModel()
{
    model_ = make_shared<Model>();

    model_->gravity = Vector3d(0., 0, -GRAVITY);

    double mass[DOF];
    mass[0] = 1.0;
    mass[1] = 1.0;
    mass[2] = 1.0;
    mass[3] = 1.0;
    mass[4] = 1.0;
    mass[5] = 1.0;
    mass[6] = 1.0;

    Vector3d axis[DOF];
	axis[0] = Eigen::Vector3d::UnitZ();
	axis[1] = Eigen::Vector3d::UnitY();
	axis[2] = Eigen::Vector3d::UnitZ();
	axis[3] = -1.0*Eigen::Vector3d::UnitY();
	axis[4] = Eigen::Vector3d::UnitZ();
	axis[5] = -1.0*Eigen::Vector3d::UnitY();
	axis[6] = -1.0*Eigen::Vector3d::UnitZ();


	Eigen::Vector3d global_joint_position[DOF];

	global_joint_position[0] = Eigen::Vector3d(0.0, 0.0, 0.3330);
	global_joint_position[1] = global_joint_position[0];
	global_joint_position[2] = Eigen::Vector3d(0.0, 0.0, 0.6490);
	global_joint_position[3] = Eigen::Vector3d(0.0825, 0.0, 0.6490);
	global_joint_position[4] = Eigen::Vector3d(0.0, 0.0, 1.0330);
	global_joint_position[5] = Eigen::Vector3d(0.0, 0.0, 1.0330);
	global_joint_position[6] = Eigen::Vector3d(0.0880, 0.0, 1.0330);

	joint_posision_[0] = global_joint_position[0];
	for (int i = 1; i < DOF; i++)
		joint_posision_[i] = global_joint_position[i] - global_joint_position[i - 1];

	com_position_[0] = Vector3d(0.000096, -0.0346, 0.2575);
	com_position_[1] = Vector3d(0.0002, 0.0344, 0.4094);
	com_position_[2] = Vector3d(0.0334, 0.0266, 0.6076);
	com_position_[3] = Vector3d(0.0331, -0.0266, 0.6914);
	com_position_[4] = Vector3d(0.0013, 0.0423, 0.9243);
	com_position_[5] = Vector3d(0.0421, -0.0103, 1.0482);
	com_position_[6] = Vector3d(0.1, -0.0120, 0.9536);

	for (int i = 0; i < DOF; i++)
		com_position_[i] -= global_joint_position[i];

    Math::Vector3d inertia[DOF];
	for (int i = 0; i < DOF; i++)
		inertia[i] = Eigen::Vector3d::Identity() * 0.001;

    for (int i = 0; i < DOF; i++) {
        body_[i] = Body(mass[i], com_position_[i], inertia[i]);
        joint_[i] = Joint(JointTypeRevolute, axis[i]);
        if (i == 0)
            body_id_[i] = model_->AddBody(0, Math::Xtrans(joint_posision_[i]), joint_[i], body_[i]);
        else
            body_id_[i] = model_->AddBody(body_id_[i - 1], Math::Xtrans(joint_posision_[i]), joint_[i], body_[i]);
    }
}

void ArmController::initFile()
{
	debug_file_.open("debug.txt");
	for (int i = 0; i < NUM_HW_PLOT; i++)
	{
		hw_plot_files_[i].open(hw_plot_file_names_[i] + ".txt");
	}
}

void ArmController::readData(const Vector7d &position, const Vector7d &velocity)
{
	for (size_t i = 0; i < dof_; i++)
	{
		q_(i) = position(i);
		q_dot_(i) = velocity(i);
		torque_(i) = 0;
	}
}
void ArmController::readData(const Vector7d &position, const Vector7d &velocity, const Vector7d &coriolis, const Vector6d &ee_force){
	if (is_wrench_modified < 10){
		for (size_t i = 0; i < 6; i++)
		{
			F_ext_Error(i) -= ee_force(i);
		}
		is_wrench_modified++;
	}
	for (size_t i = 0; i < dof_; i++)
	{
		q_(i) = position(i);
		q_dot_(i) = velocity(i);
		g_(i) = coriolis(i); 
	}
	for (size_t i = 0; i < 6; i++)
	{
		F_ext_old = F_ext_new;
		if (abs(ee_force(i)) < 10e5 && abs(ee_force(i)) > 10e-3) F_ext_new(i) = -ee_force(i) - F_ext_Error(i)/10;
	}
	//if (abs(F_ext_new(2)) >= 6) 
	F_desired << 0, 0, DESIRED_FORCE, 0, 0, 0;
	//else  F_desired = F_ext_new;
}
// TEST FOR JOINT TORQUE
void ArmController::readData(const Vector7d &position, const Vector7d &velocity, const Vector7d &ext_torque){
	Vector6d f_ext;
	f_ext = j_inverse_.transpose() * (ext_torque - torque_input);
	for (size_t i = 0; i < dof_; i++)
	{	
		double q_dot_old = q_dot_(i);
		q_(i) = position(i);
		q_dot_(i) = velocity(i);
		q_ddot_(i) = (q_dot_(i) - q_dot_old) * hz_;
		torque_(i) = 0;
	}
	for (size_t i = 0; i < 6; i++){
		F_ext_old(i) = F_ext_new(i);
		F_ext_new(i) = f_ext(i);
	}
}
// --- TEST END ---

const Vector7d & ArmController::getPositionInput()
{
	return position_input;
}

const Vector7d & ArmController::getTorqueInput()
{
	return torque_input;
}

// Debug Here
void ArmController::printState()
{
	static int DBG_CNT = 0;
	if (DBG_CNT++ > hz_ / 50.)
	{
		DBG_CNT = 0;
		if (control_mode_=="home"){
			cout << "q now :\t" << endl;
			cout << std::fixed << std::setprecision(3) << q_.transpose() << endl;
			// cout << "R log: \t" << endl;
			// cout << std::fixed << std::setprecision(3) << rotation_.log() << endl;
			// cout << "R log vec: " << rotation_.log()(2,1) <<", " << rotation_.log()(0,2) <<", "<< rotation_.log()(1,0) << endl;
			// cout << "R log vec calc: \t" << endl;
			// cout << std::fixed << std::setprecision(3) << rotation_in_vector.transpose() << endl;
			// cout << "R in vector: \t" << endl;
			// cout << std::fixed << std::setprecision(3) << RotationInVector(rotation_).transpose() << endl;
			cout << "q desired :\t" << endl;
			cout << std::fixed << std::setprecision(3) << q_desired_.transpose() << endl;
			// cout << "q_init_ :\t" << endl;
			// cout << std::fixed << std::setprecision(3) << q_init_.transpose() << endl;
			cout << "q target :\t" << endl;
			cout << std::fixed << std::setprecision(3) << q_target.transpose() << endl;
			// cout << "x current :\t" << endl;
			// cout << std::fixed << std::setprecision(3) << x_6d_.transpose() << endl;
			// cout << "q error :\t" << endl;
			// cout << std::fixed << std::setprecision(3) << (q_target-q_).transpose() << endl;
			// cout << "mass matrix :\t" << endl;
			// cout << std::fixed << std::setprecision(3) << m_ << endl;
			cout << "torque input :\t" << endl;
			cout << std::fixed << std::setprecision(3) << torque_input.transpose() << endl;
			cout << "playtime :\t" << endl;
			cout << std::fixed << std::setprecision(3) << play_time_ << endl;

		} 
		if (control_mode_=="ETank"){
			// cout << "x dot desired times gamma :\t" << endl;
			// cout << std::fixed << std::setprecision(3) << x_desired_dot_.transpose() << endl;
			cout << "x dot current :\t" << endl;
			cout << std::fixed << std::setprecision(3) << x_dot_.transpose() << endl;
			// cout << "x desired :\t" << endl;
			// cout << std::fixed << std::setprecision(3) << x_desired_6d.transpose() << endl;
			// cout << "x current :\t" << endl;
			// cout << std::fixed << std::setprecision(3) << x_6d_.transpose() << endl;
			// cout << "x init :\t" << endl;
			// cout << std::fixed << std::setprecision(3) << x_init_6d.transpose() << endl;
			// cout << "x error :\t" << endl;
			// cout << std::fixed << std::setprecision(3) << x_error_.transpose() << endl;
			cout << "tank E :\t" << endl;
			cout << std::fixed << std::setprecision(3) << tank_E << endl;
			//cout << "port power :\t" << endl;
			//cout << std::fixed << std::setprecision(3) << port_power.transpose() << endl;
			cout << "port gamma :\t" << endl;
			cout << std::fixed << std::setprecision(3) << port_gamma.transpose() << endl;
			cout << "Torque input :\t" << endl;
			cout << std::fixed << std::setprecision(3) << torque_input.transpose() << endl;
			cout << "Tau imp :\t" << endl;
			cout << std::fixed << std::setprecision(3) << tau_imp.transpose() << endl;
			cout << "F_F: \t" << endl;
			cout << std::fixed << std::setprecision(3) << F_F.transpose() << endl;	
			cout << "f_ext: \t" << endl;
			cout << std::fixed << std::setprecision(3) << F_ext_new.transpose() << endl;	
			//cout << "imp f_control: \t" << endl;
			//cout << std::fixed << std::setprecision(3) << (Kx*x_error_ + Dx*(-x_dot_control)).transpose() << endl;
			cout << "f_desired: \t" << endl;
			cout << std::fixed << std::setprecision(3) << F_desired.transpose() << endl;	
			// cout << "f_ext_Error: \t" << endl;
			// cout << std::fixed << std::setprecision(3) << F_ext_Error.transpose() << endl;	
			cout << "S_ur :\t" << endl;
			cout << std::fixed << std::setprecision(3) << S_ur << endl;		
			//cout << "playtime :\t" << endl;
			//cout << std::fixed << std::setprecision(3) << play_time_ << endl;
		}
		if (control_mode_ == "Test"){
			cout << "x dot current :\t" << endl;
			cout << std::fixed << std::setprecision(3) << x_dot_.transpose() << endl;
			cout << "x error :\t" << endl;
			cout << std::fixed << std::setprecision(3) << (x_desired_6d - x_6d_).transpose() << endl;
			cout << "F external :\t" << endl;
			cout << std::fixed << std::setprecision(3) << F_ext_new.transpose() << endl;
			cout << "Torque input :\t" << endl;
			cout << std::fixed << std::setprecision(3) << torque_input.transpose() << endl;
		}
	}
}

// Record data
void ArmController::record(int file_number, double duration)
{
	if (play_time_ < control_start_time_ + duration + 1.0)
	{
		hw_plot_files_[file_number] << x_6d_.transpose() <<
			Map< Matrix<double, 1, 9> >(rotation_.data(), rotation_.size()) <<
			endl;
	}
}
void ArmController::record(double duration)
{
	if (play_time_ < control_start_time_ + duration + 1.0)
	{
		hw_plot_files_[0] << play_time_ << " " << x_desired_dot_control(2)  << " "
		<< x_dot_(2) << " " << port_gamma(0) << " " << port_gamma(1) * port_gamma(2)  << " "<< S_ur << 
		endl;
	}
	if (play_time_ < control_start_time_ + duration + 1.0)
	{
		hw_plot_files_[1] << play_time_ << " " << port_power.transpose() << " " << tank_E <<
		endl;
	}
}


// TODO: FIX it
void ArmController::initPosition(franka::RobotState state)
{
    q_init_ = q_;
    q_desired_ = q_init_;
	model_interface_.setRobotState(state);
}

// ----------------------------------------------------

