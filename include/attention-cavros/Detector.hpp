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

#ifndef ATTENTION_CAVROS__DETECTOR_HPP_
#define ATTENTION_CAVROS__DETECTOR_HPP_

#include <string>
#include <chrono>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "gazebo_msgs/msg/model_states.hpp"

using std::placeholders::_1;

namespace attention_cavros
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

  void model_states_callback(const gazebo_msgs::msg::ModelStates::SharedPtr states);

private:
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr sub_;
  rclcpp_lifecycle::LifecyclePublisher<gazebo_msgs::msg::ModelStates>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void near_objects_publisher(void);
};

}  // namespace attention-cavros

#endif  // ATTENTION_CAVROS__DETECTOR_HPP_
