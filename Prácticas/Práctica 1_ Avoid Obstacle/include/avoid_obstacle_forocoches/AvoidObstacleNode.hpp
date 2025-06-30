// Copyright 2023 Intelligent Robotics Lab
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
/**
 * @file AvoidObstacleNode.hpp
 * @author Forocoches
 * @version 0.1
 * @date 2023-02-23
 * @copyright Copyright (c) 2023
 */
#ifndef AVOID_OBSTACLE_FOROCOCHES__AVOIDOBSTACLENODE_HPP_
#define AVOID_OBSTACLE_FOROCOCHES__AVOIDOBSTACLENODE_HPP_

#include "sensor_msgs/msg/laser_scan.hpp"
#include "kobuki_ros_interfaces/msg/button_event.hpp"
#include "kobuki_ros_interfaces/msg/bumper_event.hpp"
#include "kobuki_ros_interfaces/msg/wheel_drop_event.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "avoid_obstacle_forocoches/DebugNode.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals; // NOLINT
using std::placeholders::_1;

class AvoidObstacleNode : public rclcpp::Node
{
public:
  AvoidObstacleNode();

private:
  // Subscribtion callbacks
  void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
  void button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr msg);
  void wheel_drop_callback(kobuki_ros_interfaces::msg::WheelDropEvent::UniquePtr msg);
  void bumper_callback(kobuki_ros_interfaces::msg::BumperEvent::UniquePtr msg);

  // Control cycle
  void control_cycle();

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  DebugNode::DebugPublisher debug_pub_;

  // Subscription
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::ButtonEvent>::SharedPtr button_sub_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::WheelDropEvent>::SharedPtr wheel_drop_sub_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::BumperEvent>::SharedPtr bumper_sub_;

  // Message
  geometry_msgs::msg::Twist out_vel_;
  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;
  kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr last_button_pressed_;
  kobuki_ros_interfaces::msg::WheelDropEvent::UniquePtr last_wheel_dropped_;
  kobuki_ros_interfaces::msg::BumperEvent::UniquePtr last_bumper_detected_;
  visualization_msgs::msg::MarkerArray marker_msg_;
  DebugNode::DebugMessage debug_msg_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // --------- FSM ------------
  // FSM variables
  int state_;
  int last_state_;
  bool finished_rotation_;
  bool button_pressed_ = false;
  rclcpp::Time state_timestamp_;
  // FSM error
  bool stopped_with_error_ = false;
  bool kobuki_not_on_ground_ = false;
  const rclcpp::Duration LASER_SCAN_TIMEOUT {1s};
  // FSM states
  static const int STOP = 0;
  static const int FORWARD = 1;
  static const int TURN = 2;
  static const int ROTATION = 3;
  static const int BACKWARD = 4;  // Only used in extreme conditions
  // FSM changes
  /**
   * @brief Change the state
   * @param new_state
   */
  void change_state(int new_state);
  // FSM Check to change state
  /**
   * @brief Checks if it can resume the movement
   * @return true
   * @return false
   */
  bool check_stop_2_last();
  /**
   * @brief Checks if it needs to stop
   * @return true
   * @return false
   */
  bool check_2_stop();
  /**
   * @brief Checks if it can change from turning to rotating
   * @return true
   * @return false
   */
  bool check_turn_2_rotation();
  /**
   * @brief Check if it needs to turn
   * @return true
   * @return false
   */
  bool check_2_turn();
  /**
   * @brief Checks if the rotation has ended
   * @return true
   * @return false
   */
  bool check_rotation_2_forward();
  /**
   * @brief Checks if the turn has been triggered fast
   * @return true
   * @return false
   */
  bool check_rotation_2_turn_time();
  /**
   * @brief Check if the bumper has detected an object
   * @return true
   * @return false
   */
  bool check_2_backward();
  /**
   * @brief Checks if the backwards movement has ended
   * @return true
   * @return false
   */
  bool check_backward_2_turn();

  // --------- Velocity control ------------
  // Constants
  float SPEED_STOP_LINEAR = 0.0f;
  float SPEED_STOP_ANGULAR = 0.0f;
  float SPEED_FORWARD_LINEAR = 0.3f;
  float SPEED_FORWARD_ANGULAR = 0.0f;
  float SPEED_TURN_LINEAR = 0.0f;
  float SPEED_TURN_ANGULAR = 0.4f;
  float speed_rotation_angular_ = 0.0f;
  // Times
  const rclcpp::Duration MIN_ROTATING_TIME {1s};
  const rclcpp::Duration BACKWARD_TIME {1s};
  // Times that depend on speed
  rclcpp::Duration TURNING_TIME {1s};
  rclcpp::Duration rotating_time_ {1s};

  // --------- Obstacle control ------------
  // Sides
  const int OBJECT_IN_LEFT = -1;
  const int OBJECT_IN_RIGHT = 1;
  int obstacle_position_ = 0;
  // Constant ranges
  float OBSTACLE_DISTANCE_THRESHOLD = 1.0f;
  float MIN_THRESHOLD = 0.4f;
  float NON_DETECTION_THRESHOLD = 0.25f;
  int MIN_LASER_RANGE = 4;  // Half of the range
  // Variable ranges
  float reduced_threshold_ = 0.0f;
  float reduced_min_threshold_ = 0.3f;
  int current_range = MIN_LASER_RANGE;  // Higher = less range
  // Detection Algorithm
  /**
   * @brief Set the rotation radius
   * @param degrees: ranges index
   * @param distance
   * @return double
   */
  double set_rotation_radius(int degrees, float distance);
  /**
   * @brief Set the rotation time and speed
   * @param radius: rotation radius
   */
  void set_rotation_parameters(float radius);
  // Detection Algorithm constants
  double DETECTION_PRECISION = 0.1f;  // In rads = 5º
  double MIN_ROTATION_RADIUS = 0.5f;

  // --------- Rviz control ------------
  /**
   * @brief Set the markers array for rviz2
   */
  void set_markers();
  /**
   * @brief Set the marker for rviz2
   * @param alpha: angle
   * @param distance
   * @param id
   * @return visualization_msgs::msg::Marker
   */
  visualization_msgs::msg::Marker set_marker(float alpha, float distance, int id);
};

#endif  // AVOID_OBSTACLE_FOROCOCHES__AVOIDOBSTACLENODE_HPP_
