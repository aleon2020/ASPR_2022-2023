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

#include "seek_and_capture_forocoches/Turn.hpp"

namespace seek_and_capture_forocoches
{

using namespace std::chrono_literals;  // NOLINT

Turn::Turn(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  debug_pub_ = node_->create_publisher<DebugNode::DebugMessage>(DebugNode::TOPIC_NAME, 10);
  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("output_vel", 100);
}

void
Turn::halt()
{
}

BT::NodeStatus
Turn::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    start_time_ = node_->now();
  }

  geometry_msgs::msg::Twist vel_msgs;
  vel_msgs.angular.z = 0.3;
  vel_pub_->publish(vel_msgs);

  auto elapsed = node_->now() - start_time_;

  if (elapsed < 5s) {
    debug_msg_.data = DebugNode::PERSON_FOUND;
    debug_pub_->publish(debug_msg_);
    return BT::NodeStatus::RUNNING;
  } else {
    return BT::NodeStatus::SUCCESS;
  }
}
}  // namespace seek_and_capture_forocoches
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<seek_and_capture_forocoches::Turn>("Turn");
}
