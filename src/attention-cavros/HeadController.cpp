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

#include <string>
#include "attention-cavros/HeadController.hpp"

using namespace std::chrono_literals;

namespace attention_cavros
{

HeadControllerNode::HeadControllerNode(
  const std::string & name, const std::chrono::nanoseconds & rate)
: Node(name)
{
  sub_ = create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
    "/head_controller/state", 10,
    std::bind(&HeadControllerNode::head_state_callback, this, _1));

  pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/head_controller/joint_trajectory", 10);

  timer_ = create_wall_timer(
    rate, std::bind(&HeadControllerNode::head_publisher, this));

  i_ = 0;
  start_mov_ = now();
}

void
HeadControllerNode::head_publisher(void)
{ 
  
  float angles[5] = {75.0, 25.0, -45, 20, -75};
  if(this->now() - start_mov_ > rclcpp::Duration(2s)){
    moveHead(angles[i_],0);
    i_++;
    if(i_ == 5) i_ = 0;
  }

}

void HeadControllerNode::moveHead(float yaw, float pitch) {
  trajectory_msgs::msg::JointTrajectory message;

  float joint_yaw = yaw * 1.3 / 75;
  float joint_pitch = pitch * 0.7853 / 90;

  message.header.frame_id = "";
  message.header.stamp = this->now();
  message.joint_names = {"head_1_joint", "head_2_joint"};
                            // yaw         // pitch
  message.points.resize(1);
  message.points[0].positions.resize(2);
  message.points[0].accelerations.resize(2);
  message.points[0].velocities.resize(2);
  message.points[0].effort.resize(2);

  message.points[0].positions[0] = joint_yaw;
  message.points[0].positions[1] = joint_pitch;

  message.points[0].velocities[0] = 0.1;
  message.points[0].velocities[1] = 0.1;

  message.points[0].accelerations[0] = 0.1;
  message.points[0].accelerations[1] = 0.1;

  message.points[0].effort[0] = 0.1;
  message.points[0].effort[1] = 0.1;

  message.points[0].time_from_start = rclcpp::Duration(1s);

  start_mov_ = now();
  std::cout << "mensje enviado" <<std::endl;
  pub_->publish(message);
}


void
HeadControllerNode::head_state_callback(
  const control_msgs::msg::JointTrajectoryControllerState::SharedPtr state) 
{
  /*
  std::cout << "AAA" << std::endl;
  if ( state->error.positions[0] < 0.1 && state->error.positions[1] < 0.1 ) {
    std::cout << "bb" << std::endl;
    reached_pos_ = true;
  }
  std::cout << "NNNNN" << std::endl;
  */
}

}  // namespace attention_cavros
