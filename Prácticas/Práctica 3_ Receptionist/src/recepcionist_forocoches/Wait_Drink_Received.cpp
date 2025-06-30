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

#include "recepcionist_forocoches/Wait_Drink_Received.hpp"

namespace recepcionist_forocoches
{

using namespace std::chrono_literals;  // NOLINT
using std::placeholders::_1;

Wait_Drink_Received::Wait_Drink_Received(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  // Settling blackboard
  config().blackboard->get("node", node_);
  sound_pub_ = node_->create_publisher<kobuki_ros_interfaces::msg::Sound>("output_sound", 10);
  dialog_.registerCallback(
    std::bind(
      &Wait_Drink_Received::deliverDrinkIntentCB, this,
      _1), "Deliver Drink");
}

void Wait_Drink_Received::deliverDrinkIntentCB(
  dialogflow_ros2_interfaces::msg::DialogflowResult result)
{
  // RCLCPP_INFO(
  //  node_->get_logger(), "[ExampleDF] Wait_Drink_Received: intent [%s]", result.intent.c_str());
  responded_ = true;
  dialog_.speak(result.fulfillment_text);
  RCLCPP_INFO(node_->get_logger(), "%s\n", result.fulfillment_text.c_str());
}

void
Wait_Drink_Received::halt()
{
}

BT::NodeStatus
Wait_Drink_Received::tick()
{
  responded_ = false;
  if (status() == BT::NodeStatus::IDLE) {
    dialog_.speak("Here is your drink");
    RCLCPP_INFO(node_->get_logger(), "Here is your drink");
    out_sound_.value = 0;
    sound_pub_->publish(out_sound_);
    dialog_.listen();
  }

  rclcpp::spin_some(dialog_.get_node_base_interface());
  if (responded_) {
    responded_ = false;
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::RUNNING;
  }
}


}  // namespace recepcionist_forocoches
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist_forocoches::Wait_Drink_Received>("Wait_Drink_Received");
}
