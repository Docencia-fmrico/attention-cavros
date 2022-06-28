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

#ifndef TRACKING__HEADCONTROLLER_HPP_
#define TRACKING__HEADCONTROLLER_HPP_

#include <string>
#include <chrono>
#include <unistd.h>
#include <iostream>
//#include <time.hpp>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "gazebo_msgs/msg/link_states.hpp"

#include "ros2_knowledge_graph/GraphNode.hpp"
#include "ros2_knowledge_graph/graph_utils.hpp"


using std::placeholders::_1;

namespace tracking
{

class HeadControllerNode : public rclcpp::Node
{
public:
  HeadControllerNode(const std::string & name, const std::chrono::nanoseconds & rate);
  void init_grafLuisFonsi(void);

private:
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  //rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr gazebo_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void scan(void);
  void head_state_callback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr state) ;
  //void model_states_callback(const gazebo_msgs::msg::LinkStates::SharedPtr states);
  void moveHead(float yaw, float pitch);
  void HeadControl(void);
  
  void init_graph(void);
  void add_node(void);
  void add_edge(void);

  rclcpp::Time start_mov_ ;
  tf2::Stamped<tf2::Transform> object_tf;

  bool no_objects_;
  bool reached_pos_;
  float target_angle_;
  bool start_scan_;
  std::string object_;

  std::vector<std::string> split(std::string str, char del);
  std::vector<std::string> targets_;
  std::chrono::nanoseconds rate_;

  std::shared_ptr<ros2_knowledge_graph::GraphNode> graph_;
};

}  // namespace tracking

#endif  // TRACKING__HEADCONTROLLER_HPP_
