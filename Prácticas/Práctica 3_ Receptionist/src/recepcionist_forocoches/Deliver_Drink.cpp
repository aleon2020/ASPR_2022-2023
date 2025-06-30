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

#include "recepcionist_forocoches/Deliver_Drink.hpp"

namespace recepcionist_forocoches
{

using namespace std::chrono_literals;  // NOLINT
using std::placeholders::_1;
using std::placeholders::_2;

Deliver_Drink::Deliver_Drink(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  // Settling blackboard
  config().blackboard->get("node", node_);


  // Obtaining parameters
  tolerance_ = node_->get_parameter("navigation.tolerance").as_double();
  finished_ = false;

  // Building Action Client
  action_client_ =
    rclcpp_action::create_client<recepcionist_forocoches::Deliver_Drink::NavigateToPose>(
    node_, "navigate_to_pose");

  // Building tf listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_);

  // Building goal
  goal_ = NavigateToPose::Goal();
  goal_.pose.header.frame_id = "map";
}

void
Deliver_Drink::halt()
{
}

BT::NodeStatus
Deliver_Drink::tick()
{
  // --- Initializing Action (if inactive)---
  if (status() == BT::NodeStatus::IDLE) {
    finished_ = false;

    // --- Settling pose ---
    geometry_msgs::msg::PoseStamped wp;
    getInput("chair", wp);
    goal_.pose = wp;

    if (!action_client_->wait_for_action_server()) {
      RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    RCLCPP_INFO(
      node_->get_logger(), "Info:%lf,%lf", goal_.pose.pose.position.x,
      goal_.pose.pose.position.y);

    // --- Initializing Action ---
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      std::bind(&Deliver_Drink::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&Deliver_Drink::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&Deliver_Drink::result_callback, this, _1);

    // --- Starting Navigation ---
    RCLCPP_INFO(node_->get_logger(), "Sending goal...");
    action_client_->async_send_goal(goal_, send_goal_options);
    bt_status_ = BT::NodeStatus::RUNNING;
  }

  if (finished_) {
    return BT::NodeStatus::SUCCESS;
  }

  // --- Obtaining distance to goal ---
  try {
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      "base_footprint",
      goal_.pose.header.frame_id,
      goal_.pose.header.stamp);
    double x_coord = -transform.transform.translation.x;
    double y_coord = -transform.transform.translation.y;
    RCLCPP_INFO(node_->get_logger(), "Actual Coordinates: [%f,%f]", x_coord, y_coord);
    if (abs(goal_.pose.pose.position.x - x_coord) < tolerance_ &&
      abs(goal_.pose.pose.position.y - y_coord) < tolerance_)
    {
      bt_status_ = BT::NodeStatus::SUCCESS;
      finished_ = true;
      action_client_->async_cancel_all_goals();
      RCLCPP_INFO(node_->get_logger(), "Target Completed");
    }
  } catch (tf2::TransformException & e) {
    RCLCPP_ERROR(node_->get_logger(), "Error while looking up transform: %s", e.what());
  }

  return bt_status_;
}

void
Deliver_Drink::goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected");
    bt_status_ = BT::NodeStatus::FAILURE;
  } else {
    RCLCPP_INFO(node_->get_logger(), "Goal accepted. Navigating to waypoint");
    bt_status_ = BT::NodeStatus::RUNNING;
  }
}

void
Deliver_Drink::feedback_callback(
  GoalHandleNavigateToPose::SharedPtr goal,
  std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  (void)feedback;
}

void
Deliver_Drink::result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
{
  if (finished_) {
    bt_status_ = BT::NodeStatus::SUCCESS;
  }
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "Goal achieved!!");
      bt_status_ = BT::NodeStatus::SUCCESS;
      finished_ = true;
      return;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
      bt_status_ = BT::NodeStatus::SUCCESS;
      finished_ = true;
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
      bt_status_ = BT::NodeStatus::SUCCESS;
      finished_ = true;
      return;
    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
      bt_status_ = BT::NodeStatus::FAILURE;
      return;
  }
  bt_status_ = BT::NodeStatus::SUCCESS;
}
}  // namespace recepcionist_forocoches
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist_forocoches::Deliver_Drink>("Deliver_Drink");
}
