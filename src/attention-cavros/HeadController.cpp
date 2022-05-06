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

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"


#include <tf2_ros/buffer.h>


#include <kdl/frames.hpp>

using namespace std::chrono_literals;

//std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;


namespace attention_cavros
{

HeadControllerNode::HeadControllerNode(
  const std::string & name, const std::chrono::nanoseconds & rate)
: Node(name)
{

  pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/head_controller/joint_trajectory", 10);

  timer_ = create_wall_timer(
    rate, std::bind(&HeadControllerNode::head_publisher, this));
}



void
HeadControllerNode::head_publisher(void)
{
  RCLCPP_INFO(get_logger(), "Publishing something...");

  trajectory_msgs::msg::JointTrajectory command_msg;

  command_msg.header.frame_id = "";

  command_msg.header.stamp = this->now();
  
  
  command_msg.joint_names = {"head_1_joint","head_2_joint"};


  command_msg.points.resize(1);
  command_msg.points[0].positions.resize(2);
  command_msg.points[0].velocities.resize(2);
  command_msg.points[0].accelerations.resize(2);
  command_msg.points[0].effort.resize(2);


  command_msg.points[0].positions[0] = 1.3;
  command_msg.points[0].positions[1] = 0.0;

  // +
  // + arriba izq

  // -
  // - abajo derecha

  // +
  // - abajo izq

  // - arriba derecha
  // +
  command_msg.points[0].velocities[0] = 0.1;
  command_msg.points[0].velocities[1] = 0.1;

  command_msg.points[0].accelerations[0] = 0.1;
  command_msg.points[0].accelerations[1] = 0.1;

  command_msg.points[0].effort[0] = 0.1;
  command_msg.points[0].effort[1] = 0.1;


  command_msg.points[0].time_from_start = rclcpp::Duration(2s);

  RCLCPP_INFO(get_logger(), "ssdsssdd");


  geometry_msgs::PoseStamped msg_in,msg_out;

  double inx = 0.0,iny=0.0;
  double outx = 5.5,outy=0;


  inx = msg_in.pose.position.x;
  iny = msg_in.pose.position.y;

  outx = msg_out.pose.position.x;
  outy = msg_out.pose.position.y;

  geometry_msgs::TransformStamped transform;

  //tf2::doTransform(&msg_in, &msg_out, &transform);

  pub_->publish(command_msg);


}



}  // namespace attention_cavros
