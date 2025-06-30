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

#include "recepcionist_forocoches/Wait_Person.hpp"

namespace recepcionist_forocoches
{

using namespace std::chrono_literals;  // NOLINT
using std::placeholders::_1;

Wait_Person::Wait_Person(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  detection_sub_ = node_->create_subscription<vision_msgs::msg::Detection3DArray>(
    "output_detection_3d", rclcpp::SensorDataQoS(),
    std::bind(&Wait_Person::detection_callback, this, _1));
}

void Wait_Person::detection_callback(vision_msgs::msg::Detection3DArray::UniquePtr msg)
{
  last_detection_ = std::move(msg);
}

void
Wait_Person::halt()
{
}

BT::NodeStatus
Wait_Person::tick()
{
  RCLCPP_INFO(node_->get_logger(), "Waiting for person...");
  if (last_detection_ == nullptr) {
    return BT::NodeStatus::RUNNING;
  }

  for (auto detection : last_detection_->detections) {
    if (detection.results[0].hypothesis.class_id.compare("person") == 0) {
      if (detection.bbox.center.position.z < 1.5) {
        return BT::NodeStatus::SUCCESS;
      }
      RCLCPP_INFO(
        node_->get_logger(), "Client detected, depth %f",
        detection.bbox.center.position.z);
    }
  }

  return BT::NodeStatus::RUNNING;
}
}  // namespace recepcionist_forocoches
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist_forocoches::Wait_Person>("Wait_Person");
}
