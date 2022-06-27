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

#ifndef DETECTOR__DETECTOR_HPP_
#define DETECTOR__DETECTOR_HPP_

#include <string>
#include <chrono>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "gazebo_msgs/msg/link_states.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "ros2_knowledge_graph/GraphNode.hpp"
#include "ros2_knowledge_graph/graph_utils.hpp"

using std::placeholders::_1;

namespace detector
{

class DetectorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  DetectorNode(const std::string & name, const std::chrono::nanoseconds & rate);

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  void model_states_callback(const gazebo_msgs::msg::LinkStates::SharedPtr states);

private:
  rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr sub_;
  rclcpp_lifecycle::LifecyclePublisher<gazebo_msgs::msg::LinkStates>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<ros2_knowledge_graph::GraphNode> graph_;

  std::chrono::nanoseconds rate_;

  double detection_dist_;

  std::vector<std::string> targets_;
  std::vector<std::string> finded_targets_;
  std::vector<geometry_msgs::msg::Point> finded_coords_;

  void near_objects_publisher(void);
  std::vector<std::string> split(std::string str, char del);
};

}  // namespace detector

#endif  // DETECTOR__DETECTOR_HPP_
