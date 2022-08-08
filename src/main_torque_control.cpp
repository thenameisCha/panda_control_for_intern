
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

int ind = 0;
Eigen::Matrix<double, 6, 1> sum;

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

  robot.setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
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

    // zero-configure exernal wrench
    if (ind < 10){
      std::cout << "External Wrench Zero configuring" << std::endl;
      sum += O_F_Ext_hat_K;
      ind++;
      Eigen::Matrix<double, 7, 1> dummy;
      dummy.setZero();
      std::array<double, 7> tau_d_array{};
      Eigen::Matrix<double, 7, 1>::Map(&tau_d_array[0], 7) = dummy;
      return tau_d_array;
    }
    else if (ind >= 10){
      if (ind == 10){
        sum = sum / 10;
        std::cout << "External Wrench Zero Configured" << std::endl;
        std::cout << "Wrench Error :\t" << std::setprecision(3) << sum.transpose() << std::endl;
      }
      control_mode_mutex.lock();
      ac.readData(q,dq,coriolis, O_F_Ext_hat_K - sum);
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
    }
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
