
// Copyright (c) 2018-2022 Seoul National University - Suhan Park (psh117@snu.ac.kr) @ DYROS 
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

// Native C
#include <sys/mman.h>	// Needed for mlockall()

#include <iostream>
#include <iomanip>
#include <string>
#include <memory>
#include <thread>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>

#include "controller.h"
#include "motion_generator.h"
#include "terminal_func.h"

using namespace std;

#define MODE(X,Y) case X: ac_ptr->setMode(Y); break;

bool running = true;

ArmController *ac_ptr;
std::mutex control_mode_mutex;

void inputCollector()
{
  while(running)
  {
    if (kbhit())
    {
      int key = getchar();
      control_mode_mutex.lock();
      switch (key)
      {
        // Implement with user input
        MODE('i', "joint_ctrl_init")
        MODE('e', "ETank")
        MODE('h', "home")
        MODE('t', "Test")
      default:
        break;
      }
      control_mode_mutex.unlock();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

int main()
{
  if (mlockall(MCL_CURRENT | MCL_FUTURE))
    perror("mlockall failed:");
  
  franka::Robot robot("172.16.2.2",  franka::RealtimeConfig::kIgnore );
  franka::Model model = robot.loadModel();
  const int thread_priority = sched_get_priority_max(SCHED_FIFO);
  std::cout << thread_priority;
  sched_param thread_param{};
  thread_param.sched_priority = thread_priority;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param) != 0) {
    std::cout << "error!";
  }
  /*
  const std::array< double, 7 > &lower_torque_thresholds_acceleration, 
  const std::array< double, 7 > &upper_torque_thresholds_acceleration, 
  const std::array< double, 7 > &lower_torque_thresholds_nominal, 
  const std::array< double, 7 > &upper_torque_thresholds_nominal, 
  const std::array< double, 6 > &lower_force_thresholds_acceleration, 
  const std::array< double, 6 > &upper_force_thresholds_acceleration, 
  const std::array< double, 6 > &lower_force_thresholds_nominal, 
  const std::array< double, 6 > &upper_force_thresholds_nominal
  */
  // robot.setCollisionBehavior(
  //     {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
  //     {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
  //     {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
  //     {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});


  /////////////////////// Force limit setting //////////////////////////////////
  // Default
  // const double force_thresholds_acceleration = 20.0;
  // const double force_thresholds_nominal = 10.0;
  
  // Force * 4
  const double force_thresholds_acceleration = 30.0;
  const double force_thresholds_nominal = 40.0;
  //////////////////////////////////////////////////////////////////////////////

  const std::array<double, 6> force_thresholds_accelerations{{
    force_thresholds_acceleration,
    force_thresholds_acceleration,
    force_thresholds_acceleration,
    force_thresholds_acceleration,
    force_thresholds_acceleration,
    force_thresholds_acceleration}};

  const std::array<double, 6> force_thresholds_nominals{{
    force_thresholds_nominal,
    force_thresholds_nominal,
    force_thresholds_nominal,
    force_thresholds_nominal,
    force_thresholds_nominal,
    force_thresholds_nominal}};

  robot.setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      force_thresholds_accelerations, force_thresholds_accelerations,
      force_thresholds_nominals, force_thresholds_nominals);

  robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

  // First move the robot to a suitable joint configuration
  std::array<double, 7> q_goal = {{0.0, 0.0, 0.0, -M_PI / 2., 0.0, M_PI / 2, 0}};

  MotionGenerator motion_generator(0.5, q_goal);
  std::cout << "WARNING: This program will move the robot! "
            << "Please make sure to have the user stop button at hand!" << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore(100,'\n');
  robot.control(motion_generator);
  std::cout << "Finished moving to initial joint configuration." << std::endl;
  usleep(500000);

  const double hz = 1000.;
  const double period = 1./hz;
  ArmController ac(hz, model);
  double current_time = 0.0;
  ac_ptr = &ac;

  // TEST WITH JOINT SPACE TORQUE
  franka::RobotState initial_state = robot.readOnce();
  Eigen::VectorXd initial_tau_ext(7), tau_error_integral(7), tau_ext(7);
  // Bias torque sensor
  std::array<double, 7> gravity_array = model.gravity(initial_state);
  Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_tau_measured(initial_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_gravity(gravity_array.data());
  initial_tau_ext = initial_tau_measured - initial_gravity;
  // --- TEST END ---

  std::thread input_thread(inputCollector);

  bool is_first = true;
  
  // define callback for the torque control loop
  std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
      torque_control_callback = [&](const franka::RobotState& robot_state,
                                        franka::Duration /*duration*/) -> franka::Torques {
    // get state variables
    std::array<double, 7> coriolis_array = model.coriolis(robot_state);

    // convert to Eigen
    Eigen::Map<const Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
    Eigen::Map<const Eigen::Matrix<double, 6, 1> > O_F_Ext_hat_K(robot_state.O_F_ext_hat_K.data());

    //TEST WITH JOINT SPACE TORQUE
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_ext_hat(robot_state.tau_ext_hat_filtered.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
    tau_ext << tau_measured - gravity - initial_tau_ext;
    // --- TEST END ---

    // zero-configure exernal wrench
  
      control_mode_mutex.lock();

      ac.readData(q,dq,coriolis, O_F_Ext_hat_K);
      //ac.readData(q,dq,tau_ext);
      if (is_first) {
        ac.initPosition(robot_state);
        is_first = false;
      }
      current_time += period;
      
      ac.readCurrentTime(current_time);
      ac.compute();

      std::array<double, 7> tau_d_array{};
      Eigen::Matrix<double, 7, 1>::Map(&tau_d_array[0], 7) = ac.getTorqueInput();
      control_mode_mutex.unlock();
      // franka::Torques output(tau_d_array);
      // return output;
      return tau_d_array;
    };
  try {
    // start real-time control loop
    std::cout << "WARNING: Collision thresholds are set to high values. "
              << "Make sure you have the user stop at hand!" << std::endl
              << "After starting try to push the robot and see how it reacts." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(torque_control_callback);
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    running = false;
    input_thread.join();
    return -1;
  }
  return 0;
}