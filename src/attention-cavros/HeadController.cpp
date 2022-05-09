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
#include <vector>


namespace attention_cavros
{

HeadControllerNode::HeadControllerNode(
  const std::string & name, const std::chrono::nanoseconds & rate)
: Node(name), rate_(rate)
{
  sub_ = create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
    "/head_controller/state", 10,
    std::bind(&HeadControllerNode::head_state_callback, this, _1));

  pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/head_controller/joint_trajectory", 10);


  gazebo_sub_ = create_subscription<gazebo_msgs::msg::LinkStates>(
    "/gazebo/link_states", 10, std::bind(&HeadControllerNode::model_states_callback, this, _1));


  timer_ = create_wall_timer(
    rate, std::bind(&HeadControllerNode::head_publisher, this));
}



void
HeadControllerNode::model_states_callback(const gazebo_msgs::msg::LinkStates::SharedPtr states)
{   
  const unsigned int IDX_GENERAL_NAME = 2;

  for (int i = 0; i < states->name.size(); i++) {
    std::vector<std::string> current_str_v = split(states->name[i], ':');
  
  
    // Filter models and saves them into a vector
    for (int j = 0; j < targets_.size(); j++) {
      if (current_str_v[IDX_GENERAL_NAME] == targets_[j]) {
        // SUSTITUIR POR LA POSE DEL ROBOT CON RESPECTO AL MAPA
        int robot_x = 0, robot_y = 0;
        double circle_eq;

        circle_eq = pow(states->pose[i].position.x - robot_x, 2) +
          pow(states->pose[i].position.y - robot_y, 2);

        // 2nd Filter, if object inside detection radius, added.
        if (circle_eq <= pow(detection_dist_, 2)) {
          std::cout << "publish velocities tiago" << std::endl;
        }
      }
    }
  }
}

std::vector<std::string>
HeadControllerNode::split(std::string str, char del)
{
  std::string temp = "";
  std::vector<std::string> result;

  for (int i = 0; i < str.size(); i++) {
    if (str[i] != del) {
      temp += str[i];
    } else {
      result.push_back(temp);
      temp = "";
    }
  }
  result.push_back(temp);
  return result;
}

  


void
HeadControllerNode::head_publisher(void)
{
  RCLCPP_INFO(get_logger(), "Publishing something...");
  
  trajectory_msgs::msg::JointTrajectory command_msg;
  command_msg.header.stamp = now();
  //command_msg.joint_names = last_state_->joint_names;
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

  //command_msg.points[0].time_from_start = rclcpp::Duration(1s);

  pub_->publish(command_msg);


}

void
HeadControllerNode::head_state_callback(
  const control_msgs::msg::JointTrajectoryControllerState::SharedPtr state) const
{
  RCLCPP_INFO(get_logger(), "Recv head state...");
}

}  // namespace attention_cavros
