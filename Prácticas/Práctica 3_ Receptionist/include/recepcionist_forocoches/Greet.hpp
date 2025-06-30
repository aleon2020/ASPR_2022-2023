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

#ifndef RECEPCIONIST_FOROCOCHES__GREET_HPP_
#define RECEPCIONIST_FOROCOCHES__GREET_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "sound_play.hpp"
#include "gb_dialog/DialogInterface.hpp"

namespace recepcionist_forocoches
{
class Greet : public BT::ActionNodeBase
{
public:
  // Constructor
  explicit Greet(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  // Startup Callback
  void halt();

  // Startup Callback
  BT::NodeStatus tick();

  // BT PortsList
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("person_name"),
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_time_;
  BT::NodeStatus bt_status_;
  gb_dialog::DialogInterface dialog_;
};
}  // namespace recepcionist_forocoches

#endif  // RECEPCIONIST_FOROCOCHES__GREET_HPP_
