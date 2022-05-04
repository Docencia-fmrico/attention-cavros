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
: LifecycleNode(name)
{
  pub_ = create_publisher<gazebo_msgs::msg::LinkStates>("/near_objects/filtered", 10);
  sub_ = create_subscription<gazebo_msgs::msg::LinkStates>(
    "/gazebo/link_states", 10, std::bind(&DetectorNode::model_states_callback, this, _1));

  timer_ = create_wall_timer(
    rate, std::bind(&DetectorNode::near_objects_publisher, this));

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
  // Crear timer?
  RCLCPP_INFO(get_logger(), "[%s] On_activate desde [%s]", get_name(), state.label().c_str());
  pub_->on_activate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
DetectorNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  // Destruir timer?
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
  RCLCPP_INFO(get_logger(), "states.names: %s", states->name[20].c_str());
  RCLCPP_INFO(get_logger(), "states.pose.x: %f", states->pose[20].position.x);
  RCLCPP_INFO(get_logger(), "states.pose.y: %f", states->pose[20].position.y);
  RCLCPP_INFO(get_logger(), "states.pose.z: %f", states->pose[20].position.z);
}

}  // namespace attention_cavros
