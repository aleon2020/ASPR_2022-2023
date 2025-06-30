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

#include "seek_and_capture_forocoches/IsPerson.hpp"

namespace seek_and_capture_forocoches
{

using namespace std::chrono_literals;  // NOLINT
using std::placeholders::_1;

IsPerson::IsPerson(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  config().blackboard->get("node", node_);
  person_detected = 0;

  debug_pub_ = node_->create_publisher<DebugNode::DebugMessage>(DebugNode::TOPIC_NAME, 10);
  detection_sub_ = node_->create_subscription<vision_msgs::msg::Detection3DArray>(
    "output_detection_3d", rclcpp::SensorDataQoS(),
    std::bind(&IsPerson::detection_callback, this, _1));

  // Building tf broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
}

void IsPerson::detection_callback(vision_msgs::msg::Detection3DArray::UniquePtr msg)
{
  last_detection_ = std::move(msg);
}

void
IsPerson::halt()
{
}

bool IsPerson::tf_already_exists_in_position(tf2::Transform robot2newPerson, int persons_detected)
{
  for (int i = 0; i < persons_detected; i++) {
    // Get the tf from the person already found to the robot
    geometry_msgs::msg::TransformStamped oldPerson2robot_msg;
    tf2::Stamped<tf2::Transform> oldPerson2robot;
    try {
      oldPerson2robot_msg = tf_buffer_.lookupTransform(
        "person_detected" + std::to_string(i),
        "base_link", tf2::timeFromSec(rclcpp::Time(last_detection_->header.stamp).seconds() - 0.3));
      tf2::fromMsg(oldPerson2robot_msg, oldPerson2robot);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(), "Person transform not found: %s", ex.what());
      return true;
    }

    tf2::Transform oldPerson2newPerson = oldPerson2robot * robot2newPerson;
    geometry_msgs::msg::TransformStamped oldPerson2newPerson_msg;
    oldPerson2newPerson_msg.transform = tf2::toMsg(oldPerson2newPerson);

    if (oldPerson2newPerson_msg.transform.translation.x < 0.5 &&
      oldPerson2newPerson_msg.transform.translation.x > -0.5 &&
      oldPerson2newPerson_msg.transform.translation.y < 0.5 &&
      oldPerson2newPerson_msg.transform.translation.y > -0.5)
    {
      // Not near the other person
      return true;
    }
  }
  return false;
}

BT::NodeStatus
IsPerson::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    start_time_ = node_->now();
  }

  if (last_detection_ == nullptr) {
    debug_msg_.data = DebugNode::ERROR;
    debug_pub_->publish(debug_msg_);
    return BT::NodeStatus::FAILURE;
  }

  for (auto detection : last_detection_->detections) {
    if (detection.results[0].hypothesis.class_id.compare("person") == 0) {
      // BUG: filtering bad depth detection results
      if (detection.bbox.center.position.z < 0.5 || detection.bbox.center.position.z > 5.5) {
        continue;
      }
      // Tf from robot to person
      tf2::Transform robot2person;
      robot2person.setOrigin(
        tf2::Vector3(
          detection.bbox.center.position.z,
          -detection.bbox.center.position.x, 0.0));
      robot2person.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
      // -------
      // Check if person is already published and found
      if (!tf_already_exists_in_position(robot2person, person_detected)) {
        // Tf from odom to robot
        geometry_msgs::msg::TransformStamped odom2robot_msg;
        tf2::Stamped<tf2::Transform> odom2robot;
        try {
          odom2robot_msg = tf_buffer_.lookupTransform(
            "odom", "base_link",
            tf2::timeFromSec(rclcpp::Time(last_detection_->header.stamp).seconds() - 0.3));
          tf2::fromMsg(odom2robot_msg, odom2robot);
        } catch (tf2::TransformException & ex) {
          debug_msg_.data = DebugNode::ERROR;
          debug_pub_->publish(debug_msg_);
          return BT::NodeStatus::FAILURE;
        }
        // --------------------
        // Final tf to publish
        tf2::Transform odom2person = odom2robot * robot2person;
        geometry_msgs::msg::TransformStamped odom2person_msg;
        odom2person_msg.transform = tf2::toMsg(odom2person);

        odom2person_msg.header = detection.header;
        odom2person_msg.header.frame_id = "odom";
        odom2person_msg.child_frame_id = "person_detected" + std::to_string(person_detected);

        config().blackboard->set(
          "person_frame",
          "person_detected" + std::to_string(person_detected));
        person_detected++;
        tf_broadcaster_->sendTransform(odom2person_msg);
        debug_msg_.data = DebugNode::PERSON_DETECTED;
        debug_pub_->publish(debug_msg_);
        return BT::NodeStatus::SUCCESS;
      }
    }
  }

  return BT::NodeStatus::FAILURE;
}
}  // namespace seek_and_capture_forocoches
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<seek_and_capture_forocoches::IsPerson>("IsPerson");
}
