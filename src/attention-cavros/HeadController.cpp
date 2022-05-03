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

namespace attention-cavros
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
}

void
HeadControllerNode::head_state_callback(
  const control_msgs::msg::JointTrajectoryControllerState::SharedPtr state) const
{
  RCLCPP_INFO(get_logger(), "Recv head state...");
}

}  // namespace attention-cavros
