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

#include "recepcionist_forocoches/Get_Waypoint.hpp"

namespace recepcionist_forocoches
{

using namespace std::chrono_literals;  // NOLINT
using std::placeholders::_1;

Get_Waypoint::Get_Waypoint(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  button = false;
  button_sub_ = node_->create_subscription<kobuki_ros_interfaces::msg::ButtonEvent>(
    "input_button", 10,
    std::bind(&Get_Waypoint::button_callback, this, _1));

  geometry_msgs::msg::PoseStamped wp;
  double yaw;
  wp.header.frame_id = "map";
  tf2::Quaternion quaternion;

  // door wp
  wp.pose.position.x = node_->get_parameter("door.pose.position.x").as_double();
  wp.pose.position.y = node_->get_parameter("door.pose.position.y").as_double();

  yaw = node_->get_parameter("door.pose.orientation.w").as_double();
  quaternion.setRPY(0.0, 0.0, yaw);
  wp.pose.orientation = tf2::toMsg(quaternion);

  door_point_ = wp;

  // party wp
  wp.pose.position.x = node_->get_parameter("party.pose.position.x").as_double();
  wp.pose.position.y = node_->get_parameter("party.pose.position.y").as_double();

  yaw = node_->get_parameter("party.pose.orientation.w").as_double();
  quaternion.setRPY(0.0, 0.0, yaw);
  wp.pose.orientation = tf2::toMsg(quaternion);

  party_point_ = wp;

  // bar wp
  wp.pose.position.x = node_->get_parameter("bar.pose.position.x").as_double();
  wp.pose.position.y = node_->get_parameter("bar.pose.position.y").as_double();

  yaw = node_->get_parameter("bar.pose.orientation.w").as_double();
  quaternion.setRPY(0.0, 0.0, yaw);
  wp.pose.orientation = tf2::toMsg(quaternion);

  bar_point_ = wp;
}

void Get_Waypoint::button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr msg)
{
  button = true;
}


void
Get_Waypoint::halt()
{
}

BT::NodeStatus
Get_Waypoint::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    start_time_ = node_->now();
  }

  if (!button) {
    return BT::NodeStatus::RUNNING;
  }

  std::string id;
  getInput("wp_id", id);

  RCLCPP_INFO(node_->get_logger(), "NEXT TARGET: %s", id.c_str());
  if (id == "door") {
    setOutput("waypoint", door_point_);
  } else if (id == "party") {
    setOutput("waypoint", party_point_);
  } else if (id == "bar") {
    setOutput("waypoint", bar_point_);
  } else if (id == "person") {
    setOutput("waypoint", party_point_);
  }

  return BT::NodeStatus::SUCCESS;
}
}  // namespace recepcionist_forocoches
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist_forocoches::Get_Waypoint>("Get_Waypoint");
}
