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

#ifndef RECEPCIONIST_FOROCOCHES__DELIVER_DRINK_HPP_
#define RECEPCIONIST_FOROCOCHES__DELIVER_DRINK_HPP_

#include <memory>
#include <string>

#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

namespace recepcionist_forocoches
{

class Deliver_Drink : public BT::ActionNodeBase
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  // --- Constructor ---
  explicit Deliver_Drink(
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
      BT::InputPort<geometry_msgs::msg::PoseStamped>("chair")
    };
  }

protected:
  // Navigation Callbacks
  void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle);
  void feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void result_callback(const GoalHandleNavigateToPose::WrappedResult & result);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_time_;
  double tolerance_;
  bool finished_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  NavigateToPose::Goal goal_;
  BT::NodeStatus bt_status_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace recepcionist_forocoches

#endif  // RECEPCIONIST_FOROCOCHES__DELIVER_DRINK_HPP_
