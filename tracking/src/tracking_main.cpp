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

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "tracking/HeadController.hpp"
#include "tracking/PoseInMap.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  //auto tf_node = std::make_shared<tracking::TFNode>("tf_node", 2s);
  auto head_move_node = std::make_shared<tracking::HeadControllerNode>("head_node", 1s);
  head_move_node->init_graph();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(head_move_node);
  //executor.add_node(tf_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
