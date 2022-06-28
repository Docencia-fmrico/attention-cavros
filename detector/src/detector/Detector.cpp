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
#include <cmath>

#include "detector/Detector.hpp"

namespace detector
{

DetectorNode::DetectorNode(const std::string & name, const std::chrono::nanoseconds & rate)
: LifecycleNode(name), rate_(rate)
{
  // creamos un publicador de los objetos ya filtrados (?) y un subscriptor a link_states (objetos en el escenario)
  //pub_ = create_publisher<gazebo_msgs::msg::LinkStates>("/near_objects/filtered", 10);
  sub_ = create_subscription<gazebo_msgs::msg::LinkStates>(
    "/gazebo/link_states", 10, std::bind(&DetectorNode::model_states_callback, this, _1));

  // declaramos los parametros que utilizaremos luego
  declare_parameter("detection_distance", 0.0);
  declare_parameter("target_objects");
}

void
DetectorNode::init_graph(void)
{
  graph_ = std::make_shared<ros2_knowledge_graph::GraphNode>(shared_from_this());
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
DetectorNode::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] On_configure desde [%s]", get_name(), state.label().c_str());

  init_graph();
  // guardamos los parametros del fichero de configuración
  rclcpp::Parameter targets_param_format("target_objects", std::vector<std::string>({}));
  get_parameter("target_objects", targets_param_format);
  targets_ = targets_param_format.as_string_array();
  detection_dist_ = get_parameter("detection_distance").get_value<double>();
  RCLCPP_INFO(get_logger(), "[%f] Detection Dist", detection_dist_);
  for (int j = 0; j < targets_.size(); j++) {
    RCLCPP_INFO(get_logger(), "[%s] Targets[%d]", targets_[j].c_str(), j);
  }
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
DetectorNode::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] On_activate desde [%s]", get_name(), state.label().c_str());

  // comenzamos a ejecutar near_object_publisher (?)
  //timer_ = create_wall_timer(
  //  rate_, std::bind(&DetectorNode::near_objects_publisher, this));
  //pub_->on_activate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
DetectorNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] On_deactivate desde [%s]", get_name(), state.label().c_str());

  // terminamos con la ejecucion de near_object_publisher
  //pub_->on_deactivate();
  //timer_ = nullptr;

  return CallbackReturnT::SUCCESS;
}

void
DetectorNode::near_objects_publisher(void)
{
  //if (pub_->is_activated()) {
  //  RCLCPP_INFO(get_logger(), "Publishing detection...");
  //}
}

void
DetectorNode::model_states_callback(const gazebo_msgs::msg::LinkStates::SharedPtr states)
{
  // Indexes of the desired splits
  const unsigned int IDX_GENERAL_NAME = 2, IDX_SPECIFIC_NAME = 0;

  ros2_knowledge_graph_msgs::msg::Node new_node;
  ros2_knowledge_graph_msgs::msg::Edge new_edge;
  ros2_knowledge_graph_msgs::msg::Content object_content;
  tf2::Stamped<tf2::Transform> object_tf;
  tf2::Vector3 head_origin;
  RCLCPP_INFO(get_logger(), "Inicio del Callback");
  //tf2::Stamped<tf2::Transform> head2obj;
  for (int i = 0; i < states->name.size(); i++) {
    std::vector<std::string> current_str_v = split(states->name[i], ':');
    RCLCPP_INFO(get_logger(), "States[%d]: %s:%s:%s", i, current_str_v[0].c_str(), current_str_v[1].c_str(), current_str_v[2].c_str());
    // RCLCPP_INFO(get_logger(), "Comparing %s with tiago. Result:%d", current_str_v[0].c_str(), strcmp(current_str_v[0].c_str(), "tiago") == 0);
    // RCLCPP_INFO(get_logger(), "Comparing %s with head_1_link. Result: %d", current_str_v[2].c_str(), strcmp(current_str_v[2].c_str(), "head_1_link") == 0);
    // RCLCPP_INFO(get_logger(), "Result: %d", strcmp(current_str_v[0].c_str(), "tiago") == 0 && strcmp(current_str_v[2].c_str(), "head_1_link") == 0);
    //RCLCPP_INFO(get_logger(), "Comparing %s with link and %s with %d", current_str_v[2].c_str(), graph_->exist_node("tiago"));
    if (current_str_v[0].c_str() == "tiago" && current_str_v[2].c_str() == "head_1_link") {
      RCLCPP_INFO(get_logger(), "Añadido %s", current_str_v[0].c_str());
      new_node.node_name = "tiago";
      new_node.node_class = "Robot";
      graph_->update_node(new_node);
      head_origin = tf2::Vector3(states->pose[i].position.x, states->pose[i].position.y, states->pose[i].position.z);
      RCLCPP_INFO(get_logger(), "Head at %f,%f,%f", states->pose[i].position.x, states->pose[i].position.y, states->pose[i].position.z);
    }
    else if(strcmp(current_str_v[2].c_str(), "link") == 0 && graph_->exist_node("tiago")) {
      RCLCPP_INFO(get_logger(), "Añadido %s", current_str_v[0].c_str());
      new_node.node_name = current_str_v[0].c_str();
      new_node.node_class = "Object";
      graph_->update_node(new_node);
      RCLCPP_INFO(get_logger(), "At %f,%f,%f", states->pose[i].position.x, states->pose[i].position.y, states->pose[i].position.z);
      RCLCPP_INFO(get_logger(), "At %f,%f,%f", states->pose[i].position.x - head_origin[0], states->pose[i].position.y - head_origin[1], states->pose[i].position.z - head_origin[2]);
      object_tf.setOrigin(tf2::Vector3(states->pose[i].position.x - head_origin[0], states->pose[i].position.y - head_origin[1], states->pose[i].position.z - head_origin[2]));
      object_tf.setRotation(tf2::Quaternion(0, 0, 0, 1));
      object_content.type = 11;
      object_content.tf_value = toMsg(object_tf);
      new_edge.source_node_id = "tiago";
      new_edge.target_node_id = current_str_v[0].c_str();
      new_edge.content = object_content;
      graph_->update_edge(new_edge);
    }
    // SUSTITUIR POR LA POSE DEL ROBOT CON RESPECTO AL MAPA
    int robot_x = 0, robot_y = 0;
    double circle_eq;

    circle_eq = powf(states->pose[i].position.x - robot_x, 2) + 
                powf(states->pose[i].position.y - robot_y, 2);

    // 2nd Filter, if object inside detection radius, added.
    if (circle_eq <= powf(detection_dist_, 2)) {
      if ((targets_.size() != 0) && (find_targets(targets_, current_str_v[0].c_str()) == -1))
        break;
      object_content.type = 4;
      object_content.string_value = "able_to_see";
      new_edge.source_node_id = "tiago";
      new_edge.target_node_id = current_str_v[0].c_str();
      new_edge.content = object_content;
      graph_->update_edge(new_edge);
    }
    /*
    // Filter models and saves them into a vector
    for (int j = 0; j < targets_.size(); j++) {
      if (current_str_v[IDX_GENERAL_NAME] == targets_[j]) {
        
          geometry_msgs::msg::Point current_point;

          current_point.x = states->pose[i].position.x;
          current_point.y = states->pose[i].position.y;
          current_point.z = states->pose[i].position.z;

          finded_targets_.push_back(current_str_v[IDX_SPECIFIC_NAME]);
          finded_coords_.push_back(current_point);
        }
      }
    }
    */
  }
  /*
  // Debug
  for (int i = 0; i < finded_targets_.size(); i++) {
    RCLCPP_INFO(get_logger(), "name: %s", finded_targets_[i].c_str());
    RCLCPP_INFO(get_logger(), "\tx: %f", finded_coords_[i].x);
    RCLCPP_INFO(get_logger(), "\ty: %f", finded_coords_[i].y);
    RCLCPP_INFO(get_logger(), "\tz: %f", finded_coords_[i].z);
  }

  finded_targets_.clear();
  finded_coords_.clear();
  */
}

std::vector<std::string>
DetectorNode::split(std::string str, char del)
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

int
DetectorNode::find_targets(std::vector<std::string> targets, std::string to_find){
  for (int j = 0; j < targets.size(); j++) {
    if (to_find == targets[j]) {
      return j;
    }
  }
  return -1;
}

}  // namespace detector
