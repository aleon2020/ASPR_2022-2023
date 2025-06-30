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

#ifndef SEEK_AND_CAPTURE_FOROCOCHES__ISPERSON_HPP_
#define SEEK_AND_CAPTURE_FOROCOCHES__ISPERSON_HPP_

#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <string>
#include <iostream>
#include <memory>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "rclcpp/rclcpp.hpp"

#include "seek_and_capture_forocoches/DebugNode.hpp"

namespace seek_and_capture_forocoches
{

class IsPerson : public BT::ActionNodeBase
{
public:
  explicit IsPerson(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();
  void detection_callback(vision_msgs::msg::Detection3DArray::UniquePtr msg);

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::OutputPort<std::string>("person_frame")
      });
  }

  bool tf_already_exists_in_position(tf2::Transform robot2newPerson, int persons_detected);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_time_;
  DebugNode::DebugPublisher debug_pub_;
  DebugNode::DebugMessage debug_msg_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_sub_;
  vision_msgs::msg::Detection3DArray::UniquePtr last_detection_;
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  int person_detected;
};

}  // namespace seek_and_capture_forocoches

#endif  // SEEK_AND_CAPTURE_FOROCOCHES__ISPERSON_HPP_
