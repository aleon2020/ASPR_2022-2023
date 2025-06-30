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

#include <utility>
#include "avoid_obstacle_al/AvoidObstacleAlNode.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace avoid_obstacle_al
{

using namespace std::chrono_literals;
using std::placeholders::_1;

AvoidObstacleAlNode::AvoidObstacleAlNode():Node("avoid_obstacle_al")
{
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS(),
    std::bind(&AvoidObstacleAlNode::scan_callback, this, _1));
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
  timer_ = create_wall_timer(50ms, std::bind(&AvoidObstacleAlNode::control_cycle, this));
  last_state_ = STOP;
}

void
AvoidObstacleAlNode::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  last_scan_ = std::move(msg);
}

// Función donde se implementa la máquina de estados
// Explicación:
// 1. El robot está parado, una vez arranca el programa, el robot va hacia adelante
// 2. Si se detecta un obstáculo, el robot gira 90º sobre sí mismo.
// 3. Después, el robot realiza un arco de semicircunferencia para evitar el obstáculo
// 4. Si detecta un obstáculo mientras realiza el arco, se vuelve al paso 2.
// 5. Si ha terminado el arco completo, gira 90º y continúa hacia adelante (paso 1),
void
AvoidObstacleAlNode::control_cycle()
{
  if (last_scan_ == nullptr) {
    return;
  }
  switch (state_) {
    // Estado 0: Parado
    case STOP:
      out_vel_.linear.x = STOP_V;
      out_vel_.angular.z = STOP_W;
      if (forward()) {
        if (last_state_ == STOP) {
          new_state(FORWARD);
        } else {
          new_state(last_state_);
        }
      }
      break;
    // Estado 1: Adelante
    case FORWARD:
      out_vel_.linear.x = FORWARD_V;
      out_vel_.angular.z = 0;
      if (stop()) {
        new_state(STOP);
      }
      if (turn()) {
        new_state(TURN);
      }
      break;
    // Estado 2: Giro
    case TURN:
      out_vel_.linear.x = TURN_V;
      out_vel_.angular.z = TURN_W * obstacle_position_;
      if (stop()) {
        new_state(STOP);
      }
      if (end_turn()) {
        new_state(ROTATION);
      }
      break;
    // Estado 3: Rotación
    case ROTATION:
      out_vel_.linear.x = FORWARD_V;
      out_vel_.angular.z = -(TURN_W * obstacle_position_);
      if (stop()) {
        new_state(STOP);
      }
      if (end_rotation()) {
        new_state(FORWARD);
      }
      if (turn()) {
        new_state(TURN);
      }
      break;
    // Estado de fallo: El robot se detiene
    default:
      out_vel_.linear.x = STOP_V;
      out_vel_.angular.z = STOP_W;
      vel_pub_->publish(out_vel_);
      break;
  }
  vel_pub_->publish(out_vel_);
}

// Función que cambia el estado actual
void
AvoidObstacleAlNode::new_state(int new_state)
{
  last_state_ = state_;
  state_ = new_state;
  state_ts_ = now();
}

// Función que determina cuándo el robot va a realizar el giro
bool
AvoidObstacleAlNode::turn()
{
  size_t pos = last_scan_->ranges.size() / 2;
  return last_scan_->ranges[pos] < OBSTACLE_DISTANCE;
}

// Función que determina cuándo el robot va a detenerse
bool
AvoidObstacleAlNode::stop()
{
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed > LASER_DATA_SEC;
}

// Función que determina cuando el robot va hacia adelante
bool
AvoidObstacleAlNode::forward()
{
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed < LASER_DATA_SEC;
}

// Función que determina cuando el robot ha terminado de rotar
bool
AvoidObstacleAlNode::end_rotation()
{
  return (now() - state_ts_) > ROTATION_SEC;
}

// Función que determina el paso del estado giro al estado rotación
bool
AvoidObstacleAlNode::end_turn()
{
  return (now() - state_ts_) > TURN_SEC;
}
}  // namespace avoid_obstacle_al