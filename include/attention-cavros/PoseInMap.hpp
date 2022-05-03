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

#ifndef ATTENTION_CAVROS__POSEINMAP_HPP_
#define ATTENTION_CAVROS__POSEINMAP_HPP_

#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using std::placeholders::_1;

namespace attention_cavros
{

class TFNode : public rclcpp::Node
{
public:
  TFNode(const std::string & name, const std::chrono::nanoseconds & rate);

private:
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_;
  // Quitar timer y publicar en el callback del sub_ ?¿?¿?¿
  rclcpp::TimerBase::SharedPtr timer_;

  void tf_publisher(void);
  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr tf) const;
};

}  // namespace attention-cavros

#endif  // ATTENTION_CAVROS__POSEINMAP_HPP_
