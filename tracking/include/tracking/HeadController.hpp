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
#include "std_msgs/msg/string.hpp"

#include "ros2_knowledge_graph/GraphNode.hpp"
#include "ros2_knowledge_graph/graph_utils.hpp"


using std::placeholders::_1;

namespace tracking
{

class HeadControllerNode : public rclcpp::Node
{
public:
  HeadControllerNode(const std::string & name, const std::chrono::nanoseconds & rate);
  void init_graph(void);

private:
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr state_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::nanoseconds rate_;

  void scan(void);
  void head_state_callback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr state) ;
  void moveHead(float yaw, float pitch);
  void HeadControl(void);
  void look_at_target(void);
  void update_targets(ros2_knowledge_graph_msgs::msg::Edge new_tf,  std::string target_node_name);
  void target_object_callback (const std_msgs::msg::String::SharedPtr msg);
  std::vector<std::string> split(std::string str, char del);
  
  void add_node(void);
  void add_edge(void);

  bool reached_pos_;
  bool start_scan_;

  geometry_msgs::msg::TransformStamped  target_tf_;
  std::string                           target_object_;
  float                                 target_angle_;

  ros2_knowledge_graph_msgs::msg::Edge looking_at_;
  std::shared_ptr<ros2_knowledge_graph::GraphNode> graph_;
};

}  // namespace tracking

#endif  // TRACKING__HEADCONTROLLER_HPP_
