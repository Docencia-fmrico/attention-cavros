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
}



void
HeadControllerNode::head_publisher(void)
{
  RCLCPP_INFO(get_logger(), "Publishing something...");

  trajectory_msgs::msg::JointTrajectory command_msg;
  command_msg.header.stamp = now();
  
  if(last_state_ == nullptr){
    RCLCPP_INFO(get_logger(), "last state is empty");
    return;
  }
  else{
    command_msg.joint_names = last_state_->joint_names;
    RCLCPP_INFO(get_logger(), "SUCCESSSS");

  }

  command_msg.points.resize(1);
  command_msg.points[0].positions.resize(2);
  command_msg.points[0].velocities.resize(2);
  command_msg.points[0].accelerations.resize(2);

  command_msg.points[0].positions[0] = 0.0;
  command_msg.points[0].positions[1] = 0.0;

  command_msg.points[0].velocities[0] = 0.1;
  command_msg.points[0].velocities[1] = 0.1;

  command_msg.points[0].accelerations[0] = 0.1;
  command_msg.points[0].accelerations[1] = 0.1;

  command_msg.points[0].time_from_start = rclcpp::Duration(1);

  RCLCPP_INFO(get_logger(), "ssdsssdd");

  pub_->publish(command_msg);


}

void
HeadControllerNode::head_state_callback(
   control_msgs::msg::JointTrajectoryControllerState::UniquePtr state) 
{
  RCLCPP_INFO(get_logger(), "Recv head state...");
  
  last_state_ = std::move(state);//copiar la direccion de memoria y eliminarla de la cola
  
  RCLCPP_INFO(get_logger(), "WWWWWWW");


}

}  // namespace attention_cavros
