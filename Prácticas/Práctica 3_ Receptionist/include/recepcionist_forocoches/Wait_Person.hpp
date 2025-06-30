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

#ifndef RECEPCIONIST_FOROCOCHES__WAIT_PERSON_HPP_
#define RECEPCIONIST_FOROCOCHES__WAIT_PERSON_HPP_

#include <string>
#include <iostream>
#include <memory>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "vision_msgs/msg/detection3_d_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace recepcionist_forocoches
{

class Wait_Person : public BT::ActionNodeBase
{
public:
  explicit Wait_Person(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();
  void detection_callback(vision_msgs::msg::Detection3DArray::UniquePtr msg);

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_time_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_sub_;
  vision_msgs::msg::Detection3DArray::UniquePtr last_detection_;
};

}  // namespace recepcionist_forocoches

#endif  // RECEPCIONIST_FOROCOCHES__WAIT_PERSON_HPP_
