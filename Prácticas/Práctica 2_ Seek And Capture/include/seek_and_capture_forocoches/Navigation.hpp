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

#ifndef SEEK_AND_CAPTURE_FOROCOCHES__NAVIGATION_HPP_
#define SEEK_AND_CAPTURE_FOROCOCHES__NAVIGATION_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>
#include <iostream>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "seek_and_capture_forocoches/PIDController.hpp"
#include "seek_and_capture_forocoches/DebugNode.hpp"

namespace seek_and_capture_forocoches
{

class Navigation : public BT::ActionNodeBase
{
public:
  explicit Navigation(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("person_frame")
      });
  }

private:
  std::string person_frame_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_time_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  DebugNode::DebugPublisher debug_pub_;
  DebugNode::DebugMessage debug_msg_;
  PIDController * pid_lin_;
  PIDController * pid_ang_;
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace seek_and_capture_forocoches

#endif  // SEEK_AND_CAPTURE_FOROCOCHES__NAVIGATION_HPP_
