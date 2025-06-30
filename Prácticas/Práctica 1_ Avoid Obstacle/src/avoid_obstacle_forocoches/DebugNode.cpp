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
 * @file DebugNode.cpp
 * @author Javier Izquierdo Hernandez (j.izquierdoh.2021@alumnos.urjc.es)
 * @brief Node implementation for debugging purposes for the kobuki
 * @note Uses leds, sound and terminal functions for debugging
 * @version 1.0
 * @date 2023-02-17
 * @copyright Copyright (c) 2023
 */
#include "avoid_obstacle_forocoches/DebugNode.hpp"

using namespace std::chrono_literals; // NOLINT
using std::placeholders::_1;

const char * DebugNode::TOPIC_NAME = "debug_topic";

DebugNode::DebugNode()
: Node("debug_node")
{
  led1_pub_ = create_publisher<kobuki_ros_interfaces::msg::Led>("output_led_1", 10);
  led2_pub_ = create_publisher<kobuki_ros_interfaces::msg::Led>("output_led_2", 10);
  sound_pub_ = create_publisher<kobuki_ros_interfaces::msg::Sound>("output_sound", 10);

  debug_msg_sub_ = create_subscription<std_msgs::msg::Int32>(
    DebugNode::TOPIC_NAME, 10,
    std::bind(&DebugNode::debug_callback, this, _1));

  timer_ = create_wall_timer(
    500ms, std::bind(&DebugNode::control_cycle, this));
}

void DebugNode::debug_callback(std_msgs::msg::Int32::UniquePtr debug_msg)
{
  last_debug_msg_ = std::move(debug_msg);
}


void DebugNode::control_cycle()
{
  if (!debug_is_enabled && last_debug_msg_->data == OFF) {
    return;
  } else {
    debug_is_enabled = true;
  }

  const bool FLASHING = true;  // Constant to indicate the led flashing

  switch (last_debug_msg_->data) {
    case OFF:
      set_led1_value(LED_OFF);
      set_led2_value(LED_OFF);
      RCLCPP_DEBUG(get_logger(), "OFF");
      debug_is_enabled = false;
      break;
    case OK:
      set_led1_value(LED_GREEN);
      set_led2_value(LED_GREEN);
      RCLCPP_DEBUG(get_logger(), "OK");
      break;
    case READY:
      set_led1_value(LED_GREEN, FLASHING);
      set_led2_value(LED_GREEN, FLASHING);
      RCLCPP_DEBUG(get_logger(), "READY");
      break;
    case NOT_ON_GROUND:
      set_led1_value(LED_RED);
      set_led2_value(LED_RED);
      set_sound(SOUND_ERROR);
      RCLCPP_DEBUG(get_logger(), "NOT ON GROUND");
      break;
    case ERROR:
      set_led1_value(LED_RED, FLASHING);
      set_led2_value(LED_RED, FLASHING);
      set_sound(SOUND_ERROR);
      RCLCPP_DEBUG(get_logger(), "ERROR");
      break;
    default:
      RCLCPP_DEBUG(get_logger(), "NOT DEFINED");
  }

  // Publish to the leds
  led1_pub_->publish(out_led1_);
  led2_pub_->publish(out_led2_);
}

void DebugNode::set_led1_value(int color, bool flashing)
{
  out_led1_.value = color;  // Setting LED color

  /*Set if the led is flashing*/
  if (flashing) {
    led1_on_ = !led1_on_;  // Flashing led

    /*If the led needs to flash turn off led*/
    if (!led1_on_) {
      out_led1_.value = LED_OFF;
    }
  } else {
    led1_on_ = true;  // Not flashing
  }
}

void DebugNode::set_led2_value(int color, bool flashing)
{
  out_led2_.value = color;  // Setting LED color

  /*Set if the led is flashing*/
  if (flashing) {
    led2_on_ = !led2_on_;  // Flashing led

    /*If the led needs to flash turn off led*/
    if (!led2_on_) {
      out_led2_.value = LED_OFF;
    }
  } else {
    led2_on_ = true;  // Not flashing
  }
}

void DebugNode::set_sound(int sound)
{
  out_sound_.value = sound;
  // Publish to the sound
  sound_pub_->publish(out_sound_);
}
