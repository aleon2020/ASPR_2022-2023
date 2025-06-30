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

#ifndef RECEPCIONIST_FOROCOCHES__FIND_CHAIR_HPP_
#define RECEPCIONIST_FOROCOCHES__FIND_CHAIR_HPP_

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

#include "geometry_msgs/msg/twist.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "kobuki_ros_interfaces/msg/sound.hpp"

namespace recepcionist_forocoches
{

class Find_Chair : public BT::ActionNodeBase
{
public:
  explicit Find_Chair(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();
  void detection_callback(vision_msgs::msg::Detection3DArray::UniquePtr msg);

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("chair")
      });
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  vision_msgs::msg::Detection3DArray::UniquePtr last_detection_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::Sound>::SharedPtr sound_pub_;
  kobuki_ros_interfaces::msg::Sound out_sound_;
  bool found_chair;
};

}  // namespace recepcionist_forocoches

#endif  // RECEPCIONIST_FOROCOCHES__FIND_CHAIR_HPP_
