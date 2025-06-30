// Copyright 2021 Intelligent Robotics Lab
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

#ifndef RECEPCIONIST_FOROCOCHES__GET_WAYPOINT_HPP_
#define RECEPCIONIST_FOROCOCHES__GET_WAYPOINT_HPP_

#include <string>

#include "tf2/LinearMath/Quaternion.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "kobuki_ros_interfaces/msg/button_event.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/rclcpp.hpp"

namespace recepcionist_forocoches
{

class Get_Waypoint : public BT::ActionNodeBase
{
public:
  explicit Get_Waypoint(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr msg);
  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("wp_id"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("waypoint")
      });
  }

private:
  static bool params_declared;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_time_;
  geometry_msgs::msg::PoseStamped door_point_;
  geometry_msgs::msg::PoseStamped party_point_;
  geometry_msgs::msg::PoseStamped bar_point_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::ButtonEvent>::SharedPtr button_sub_;

  bool button = true;
};

}  // namespace recepcionist_forocoches

#endif  // RECEPCIONIST_FOROCOCHES__GET_WAYPOINT_HPP_
