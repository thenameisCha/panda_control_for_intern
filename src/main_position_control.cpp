
// Copyright (c) 2018-2022 Seoul National University - Suhan Park (psh117@snu.ac.kr) @ DYROS 
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

// Native C
#include <sys/mman.h>	// Needed for mlockall()

#include <iostream>
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
        MODE('h', "joint_ctrl_home")
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
  // define callback for the position control loop
  std::function<franka::JointPositions(const franka::RobotState&, franka::Duration)>
      position_control_callback = [&](const franka::RobotState& robot_state,
                                        franka::Duration duration) -> franka::JointPositions {

    Eigen::Map<const Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());

    control_mode_mutex.lock();
    ac.readData(q,dq);
    if (is_first) {
      ac.initPosition(robot_state);
      std::cout << "init position" << std::endl <<
      q.transpose() << std:: endl;
      is_first = false;
    }
    current_time += duration.toSec();

    ac.readCurrentTime(current_time);
    ac.compute();

    std::array<double, 7> position_d_array{};
    Eigen::Matrix<double, 7, 1>::Map(position_d_array.data()) = ac.getPositionInput();
    control_mode_mutex.unlock();
    franka::JointPositions output(position_d_array);
    return output;
  };

  try {
    // start real-time control loop
    std::cout << "WARNING: Collision thresholds are set to high values. "
              << "Make sure you have the user stop at hand!" << std::endl
              << "After starting try to push the robot and see how it reacts." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    // TODO: position mode : 
    robot.control(position_control_callback);

  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    running = false;
    input_thread.join();
    return -1;
  }
  return 0;
}
