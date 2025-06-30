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

#include <string>
#include <iostream>

#include "seek_and_capture_forocoches/Navigation.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace seek_and_capture_forocoches
{

using namespace std::chrono_literals;  // NOLINT

Navigation::Navigation(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  config().blackboard->get("node", node_);

  debug_pub_ = node_->create_publisher<DebugNode::DebugMessage>(DebugNode::TOPIC_NAME, 10);
  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("output_vel", 100);
  pid_lin_ = new PIDController(0.0, 5.0, 0.0, 0.5);
  pid_ang_ = new PIDController(0.0, M_PI / 2, 0.0, 0.5);
}

void
Navigation::halt()
{
}

BT::NodeStatus
Navigation::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    start_time_ = node_->now();
  }

  // Obtain person frame
  config().blackboard->get("person_frame", person_frame_);
  geometry_msgs::msg::TransformStamped robot2person;

  try {
    robot2person = tf_buffer_.lookupTransform(
      "base_link", person_frame_, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(node_->get_logger(), "Person transform not found: %s", ex.what());
    debug_msg_.data = DebugNode::ERROR;
    debug_pub_->publish(debug_msg_);
    return BT::NodeStatus::FAILURE;
  }

  // Get distance
  double x = robot2person.transform.translation.x;
  double y = robot2person.transform.translation.y;
  double length = sqrt(x * x + y * y);
  double theta = atan2(y, x);

  // Send velocity
  geometry_msgs::msg::Twist vel_msgs;
  vel_msgs.linear.x = std::clamp(pid_lin_->get_output(length - 1.0), -0.5, 0.5);
  vel_msgs.angular.z = std::clamp(pid_ang_->get_output(theta), -0.5, 0.5);

  if (vel_msgs.linear.x > 0.3) {
    debug_msg_.data = DebugNode::MOVING_MODERATE;
  } else if (vel_msgs.linear.x > 0.15) {
    debug_msg_.data = DebugNode::MOVING_SLOW;
  } else {
    debug_msg_.data = DebugNode::MOVING_REALLY_SLOW;
  }
  debug_pub_->publish(debug_msg_);
  vel_pub_->publish(vel_msgs);


  auto elapsed = node_->now() - start_time_;
  if (vel_msgs.linear.x > 0.01 || vel_msgs.linear.x < -0.01) {
    return BT::NodeStatus::RUNNING;
  } else {
    return BT::NodeStatus::SUCCESS;
  }
}
}  // namespace seek_and_capture_forocoches
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<seek_and_capture_forocoches::Navigation>("Navigate");
}
