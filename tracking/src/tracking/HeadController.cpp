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
#include "tracking/HeadController.hpp"

#define PI 3.14159
#define STRING 4
#define TF 11

using namespace std::chrono_literals;

namespace tracking
{

HeadControllerNode::HeadControllerNode(
  const std::string & name, const std::chrono::nanoseconds & rate)
: Node(name)
{
  
  sub_ = create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
    "/head_controller/state", 10,
    std::bind(&HeadControllerNode::head_state_callback, this, _1));
  
  pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/head_controller/joint_trajectory", 10);

  timer_ = create_wall_timer(
    rate, std::bind(&HeadControllerNode::HeadControl, this));

  start_mov_ = now();
  no_objects_ = true;
  reached_pos_ = true;
  start_scan_ = true;

}

void HeadControllerNode::init_grafLuisFonsi(void){
  init_graph();
  add_node();
  add_edge();
}

void
HeadControllerNode::init_graph(void)
{
  graph_ = std::make_shared<ros2_knowledge_graph::GraphNode>(shared_from_this());
}

void
HeadControllerNode::add_node(void)
{
  ros2_knowledge_graph_msgs::msg::Node new_node;

  new_node.node_name = "chair";
  new_node.node_class = "object";
  graph_->update_node(new_node);

  new_node.node_name = "tiago";
  new_node.node_class = "Robot";
  graph_->update_node(new_node);

  new_node.node_name = "can";
  new_node.node_class = "object";
  graph_->update_node(new_node);
}

void
HeadControllerNode::add_edge(void)
{
  ros2_knowledge_graph_msgs::msg::Edge new_edge;
  ros2_knowledge_graph_msgs::msg::Content object_content ;

  new_edge.source_node_id = "tiago";
  new_edge.target_node_id = "chair";

  object_content.type = STRING;
  object_content.string_value = "able_to_see";
  new_edge.content = object_content;
  graph_->update_edge(new_edge);

  new_edge.source_node_id = "tiago";
  new_edge.target_node_id = "can";

  object_content.type = STRING;
  object_content.string_value = "able_to_see";
  new_edge.content = object_content;
  graph_->update_edge(new_edge);
}

void HeadControllerNode::HeadControl(void) {

  std::vector<ros2_knowledge_graph_msgs::msg::Edge> edges = graph_->get_edges();

  for ( int i = 0; i < edges.size() ; i++) {
    if (edges[i].content.type == STRING && edges[i].content.string_value == "able_to_see"){
      std::cout << edges[i].target_node_id << std::endl;
    } else {
      std::cout << "SCAN" << std::endl;
      scan();
    }
  }
}

void HeadControllerNode::scan(void)
{ 
  if(start_scan_) {
    start_scan_ = false;
    target_angle_ = 90;
  }

  if(no_objects_) {
    if(reached_pos_) {
      target_angle_ = target_angle_ * -1;
      moveHead(target_angle_,0);
      reached_pos_ = false;
    }
  }
}

void HeadControllerNode::moveHead(float yaw, float pitch) {

  trajectory_msgs::msg::JointTrajectory message;

  float joint_yaw = yaw * (1.3 / 75);
  float joint_pitch = pitch * 0.7853 / 90;

  message.header.frame_id = "";
  message.header.stamp = this->now();
  message.joint_names = {"head_1_joint", "head_2_joint"};
                            // yaw         // pitch
  message.points.resize(1);
  message.points[0].positions.resize(2);
  message.points[0].accelerations.resize(2);
  message.points[0].velocities.resize(2);
  message.points[0].effort.resize(2);

  message.points[0].positions[0] = joint_yaw;
  message.points[0].positions[1] = joint_pitch;

  message.points[0].velocities[0] = 0.1;
  message.points[0].velocities[1] = 0.1;

  message.points[0].accelerations[0] = 0.1;
  message.points[0].accelerations[1] = 0.1;

  message.points[0].effort[0] = 0.1;
  message.points[0].effort[1] = 0.1;

  message.points[0].time_from_start = rclcpp::Duration(1s);

  start_mov_ = now();
  pub_->publish(message);

  std::cout << "mensje enviado" <<std::endl;
}

/*
void
HeadControllerNode::model_states_callback(const gazebo_msgs::msg::LinkStates::SharedPtr states)
{ 
  // ande va esto
  //object_tf = grafo_->get_edge("tiago",object_,11);

  const unsigned int IDX_GENERAL_NAME = 2;
  for (int i = 0; i < states->name.size(); i++) {
    std::vector<std::string> current_str_v = split(states->name[i], ':');
  
    // Filter models and saves them into a vector
    for (int j = 0; j < targets_.size(); j++) {
      if (current_str_v[IDX_GENERAL_NAME] == targets_[j]) {
        no_objects_ = false;

        // SUSTITUIR POR LA POSE DEL ROBOT CON RESPECTO AL MAPA
        int robot_x = 0, robot_y = 0;
        double circle_eq;

        circle_eq = pow(states->pose[i].position.x - robot_x, 2) +
          pow(states->pose[i].position.y - robot_y, 2);

        // 2nd Filter, if object inside detection radius, added.
        if (circle_eq <= pow(5, 2)) {
          
          tf2::Vector3 pos = object_tf.getOrigin();
          float angle = 360 * atan(pos[1]/pos[0])/ (2*PI);
          //std::cout << pos[0]<< "," << pos[1] << " : angle-> " << angle << std::endl;
          
          if(this->now() - start_mov_ > rclcpp::Duration(1s)){
            moveHead(angle,0);
          }

        } else {
          no_objects_ = true;
        }
      }
    }
  }
}
*/
std::vector<std::string>
HeadControllerNode::split(std::string str, char del)
{
  std::string temp = "";
  std::vector<std::string> result;

  for (int i = 0; i < str.size(); i++) {
    if (str[i] != del) {
      temp += str[i];
    } else {
      result.push_back(temp);
      temp = "";
    }
  }
  result.push_back(temp);
  return result;
}

void
HeadControllerNode::head_state_callback(
  const control_msgs::msg::JointTrajectoryControllerState::SharedPtr state) 
{
  float error = fabs(state->actual.positions[0] - (target_angle_*PI/180));
  if ( error < 0.27 ) {
    reached_pos_ = true;
  }
  
}

}  // namespace tracking
