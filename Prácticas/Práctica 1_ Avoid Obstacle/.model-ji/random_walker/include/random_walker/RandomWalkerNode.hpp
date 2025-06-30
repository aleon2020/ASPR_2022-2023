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

#ifndef RANDOM_WALKER__RANDOMWALKERNODE_HPP_
#define RANDOM_WALKER__RANDOMWALKERNODE_HPP_

#include "sensor_msgs/msg/laser_scan.hpp"
#include "kobuki_ros_interfaces/msg/button_event.hpp"
#include "kobuki_ros_interfaces/msg/bumper_event.hpp"
#include "kobuki_ros_interfaces/msg/wheel_drop_event.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "random_walker/DebugNode.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals; // NOLINT
using std::placeholders::_1;

class RandomWalkerNode : public rclcpp::Node
{
public:
  RandomWalkerNode();

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
  DebugNode::DebugMessage debug_msg_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // --------- FSM ------------
  // FSM variables
  int state_;
  int last_state_;
  bool finished_rotation_;
  bool stopped_with_error_ = false;
  bool button_pressed_ = false;
  bool kobuki_not_on_ground_ = false;
  rclcpp::Time state_timestamp_;
  // FSM states
  static const int STOP = 0;
  static const int FORWARD = 1;
  static const int TURN = 2;
  static const int ROTATION = 3;
  // Only used in extreme conditions
  static const int BACKWARD = 4;
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
   * @brief Set the rotation time
   * @param speed: rotation speed
   */
  void set_rotation_time(float speed);
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

  // Velocity control
  float SPEED_STOP_LINEAR = 0.0f;
  float SPEED_STOP_ANGULAR = 0.0f;
  float SPEED_FORWARD_LINEAR = 0.3f;
  float SPEED_FORWARD_ANGULAR = 0.0f;
  float SPEED_TURN_LINEAR = 0.0f;
  float SPEED_TURN_ANGULAR = 0.3f;
  float speed_rotation_angular_ = 0.0f;
  const rclcpp::Duration TURNING_TIME {6s};
  const rclcpp::Duration MIN_ROTATING_TIME {1s};
  rclcpp::Duration ROTATING_TIME {12s};
  const rclcpp::Duration BACKWARD_TIME {1s};

  // Laser control
  float OBSTACLE_DISTANCE_THRESHOLD = 1.0f;
  int MIN_LASER_RANGE = 4;  // Half of the range
  float MIN_THRESHOLD = 0.4f;
  float NON_DETECTION_THRESHOLD = 0.3f;
  int current_range = MIN_LASER_RANGE;  // Higher = less range
  int obstacle_position_ = 0;  // -1 Left / 1 Right
  const rclcpp::Duration LASER_SCAN_TIMEOUT {1s};
};

#endif  // RANDOM_WALKER__RANDOMWALKERNODE_HPP_
