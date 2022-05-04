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
#include <vector>
#include "attention-cavros/Detector.hpp"

namespace attention_cavros
{

DetectorNode::DetectorNode(const std::string & name, const std::chrono::nanoseconds & rate)
: LifecycleNode(name), rate_(rate)
{
  pub_ = create_publisher<gazebo_msgs::msg::LinkStates>("/near_objects/filtered", 10);
  sub_ = create_subscription<gazebo_msgs::msg::LinkStates>(
    "/gazebo/link_states", 10, std::bind(&DetectorNode::model_states_callback, this, _1));

  declare_parameter("detection_distance", 0.0);
  declare_parameter("target_objects");
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
DetectorNode::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] On_configure desde [%s]", get_name(), state.label().c_str());

  rclcpp::Parameter targets_param_format("target_objects", std::vector<std::string>({}));
  get_parameter("target_objects", targets_param_format);
  targets_ = targets_param_format.as_string_array();
  detection_dist_ = get_parameter("detection_distance").get_value<double>();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
DetectorNode::on_activate(const rclcpp_lifecycle::State & state)
{
  timer_ = create_wall_timer(
    rate_, std::bind(&DetectorNode::near_objects_publisher, this));
    
  RCLCPP_INFO(get_logger(), "[%s] On_activate desde [%s]", get_name(), state.label().c_str());
  pub_->on_activate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
DetectorNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  timer_ = nullptr;
  RCLCPP_INFO(get_logger(), "[%s] On_deactivate desde [%s]", get_name(), state.label().c_str());
  pub_.reset();
  pub_->on_deactivate();

  return CallbackReturnT::SUCCESS;
}

void
DetectorNode::near_objects_publisher(void)
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  if (pub_->is_activated()) {
    RCLCPP_INFO(get_logger(), "Publishing detection...");
  }
}

void
DetectorNode::model_states_callback(const gazebo_msgs::msg::LinkStates::SharedPtr states)
{
  const unsigned int IDX_GENERAL_NAME = 2,
                     IDX_SPECIFIC_NAME = 0;  // Index of the desired splits

  if (finded_targets_.empty()){
    for (int i = 0; i < states->name.size(); i++){
      std::vector<std::string> current_str_v = split(states->name[i], ':');

      // Filter models and saves them into a vector
      for (int j = 0; j < targets_.size(); j++){
        if (current_str_v[IDX_GENERAL_NAME] == targets_[j]){
          finded_targets_.push_back(current_str_v[IDX_SPECIFIC_NAME]);
        }
      }
    }
  }
}

std::vector<std::string> 
DetectorNode::split(std::string str, char del)
{
  std::string temp = "";
  std::vector<std::string> result;

  for (int i = 0; i < (int)str.size(); i++){
    if(str[i] != del){
      temp += str[i];
    } else {
      result.push_back(temp);
      temp = "";
    }
  }
  result.push_back(temp);
  return result;
}

}  // namespace attention_cavros
