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
 * @file DebugNode.hpp
 * @author Javier Izquierdo Hernandez (j.izquierdoh.2021@alumnos.urjc.es)
 * @brief Node implementation for debugging purposes for the kobuki
 * @note Uses leds, sound and terminal functions for debugging
 * @version 1.0
 * @date 2023-02-17
 * @copyright Copyright (c) 2023
 */
#include "kobuki_ros_interfaces/msg/led.hpp"
#include "kobuki_ros_interfaces/msg/sound.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rclcpp/rclcpp.hpp"

#ifndef RANDOM_WALKER__DEBUGNODE_HPP_
#define RANDOM_WALKER__DEBUGNODE_HPP_

using namespace std::chrono_literals; // NOLINT
using std::placeholders::_1;

class DebugNode : public rclcpp::Node
{
public:
  // Constructor
  DebugNode();

  // Debug topic
  static const char * TOPIC_NAME;

  // Debug aliases
  typedef rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr DebugPublisher;
  typedef std_msgs::msg::Int32 DebugMessage;

  // Debug codes
  static const int OFF = 1;
  static const int OK = 9;
  static const int READY = 17;
  static const int NOT_ON_GROUND = 41;
  static const int ERROR = 49;

private:
  /**
   * @brief Receives debug messages, and saves them to last_debug_msg_
   * @param debug_msg Message received from other nodes
   */
  void debug_callback(std_msgs::msg::Int32::UniquePtr debug_msg);

  /**
   * @brief Display the correct debug message according to the one received
   */
  void control_cycle();

  /**
   * @brief Set the led 1 values
   * @param color Color of the led OFF/GREEN/ORANGE/RED
   * @param flashing If the led is flashing or not
   */
  void set_led1_value(int color, bool flashing = false);

  /**
   * @brief Set the led 2 values
   * @param color Color of the led OFF/GREEN/ORANGE/RED
   * @param flashing If the led is flashing or not
   */
  void set_led2_value(int color, bool flashing = false);

  /**
   * @brief Set the sound and publish it
   * @param sound Type of sound
   */
  void set_sound(int sound);

  // Publishers
  rclcpp::Publisher<kobuki_ros_interfaces::msg::Led>::SharedPtr led1_pub_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::Led>::SharedPtr led2_pub_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::Sound>::SharedPtr sound_pub_;

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr debug_msg_sub_;

  // Messages
  kobuki_ros_interfaces::msg::Led out_led1_;
  kobuki_ros_interfaces::msg::Led out_led2_;
  kobuki_ros_interfaces::msg::Sound out_sound_;
  std_msgs::msg::Int32::UniquePtr last_debug_msg_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Debug status
  bool debug_is_enabled = false;

  // ------- Led ---------
  // Led color values
  static const int LED_OFF = 0;
  static const int LED_GREEN = 1;
  static const int LED_ORANGE = 2;
  static const int LED_RED = 3;
  // Led status
  bool led1_on_ = true;
  bool led2_on_ = true;

  // ------ Sound --------
  // Sound values
  static const int SOUND_ON = 0;
  static const int SOUND_OFF = 1;
  static const int SOUND_RECHARGE = 2;
  static const int SOUND_BUTTON = 3;
  static const int SOUND_ERROR = 4;
  static const int SOUND_CLEANINGSTART = 5;
  static const int SOUND_CLEANINGEND = 6;
};

#endif  // RANDOM_WALKER__DEBUGNODE_HPP_
