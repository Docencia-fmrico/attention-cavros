// Copyright 2022 csanrod
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ATTENTION_CAVROS__HEADCONTROLLER_HPP_
#define ATTENTION_CAVROS__HEADCONTROLLER_HPP_

#include <string>
#include <chrono>
#include <unistd.h>
#include <iostream>
//#include <time.hpp>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"

using std::placeholders::_1;

namespace attention_cavros
{

class HeadControllerNode : public rclcpp::Node
{
public:
  HeadControllerNode(const std::string & name, const std::chrono::nanoseconds & rate);
  void moveHead(float yaw, float pitch);
  
private:
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void head_publisher(void);
  void head_state_callback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr state) ;
  
  
  rclcpp::Time start_mov_ ;
  int i_;
};

}  // namespace attention_cavros

#endif  // ATTENTION_CAVROS__HEADCONTROLLER_HPP_
