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
#include "tracking/PoseInMap.hpp"

namespace tracking
{

TFNode::TFNode(const std::string & name, const std::chrono::nanoseconds & rate)
: Node(name)
{
  sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", 10, std::bind(&TFNode::tf_callback, this, _1));

  pub_ = create_publisher<tf2_msgs::msg::TFMessage>(
    "/pose_in_map", 10);

  timer_ = create_wall_timer(
    rate, std::bind(&TFNode::tf_publisher, this));
}

void
TFNode::tf_publisher(void)
{
  RCLCPP_INFO(get_logger(), "Publishing tf's...");
}

void
TFNode::tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr tf) const
{
  RCLCPP_INFO(get_logger(), "Recv tf's...");
}

}  // namespace tracking
