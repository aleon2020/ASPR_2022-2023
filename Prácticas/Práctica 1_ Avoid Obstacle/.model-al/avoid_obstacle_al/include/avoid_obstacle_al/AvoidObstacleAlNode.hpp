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

#ifndef AVOID_OBSTACLE_AL__AVOIDOBSTACLEALNODE_HPP_
#define AVOID_OBSTACLE_AL__AVOIDOBSTACLEALNODE_HPP_

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace avoid_obstacle_al
{

using namespace std::chrono_literals;

class AvoidObstacleAlNode : public rclcpp::Node
{
public:

  AvoidObstacleAlNode();

private:

  void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
  
  // Función en la que se realiza la máquina de estados
  void control_cycle();

  // Estados para la máquina de estados
  static const int STOP = 0;
  static const int FORWARD = 1;
  static const int TURN = 2;
  static const int ROTATION = 3;

  // Variables utilizadas para los cambios de estado
  int state_;
  int last_state_;
  rclcpp::Time state_ts_;

  // Funciones 
  void new_state(int new_state);
  bool turn();
  bool stop();
  bool end_rotation();
  bool end_turn();
  bool forward();

  // Parámetros referentes a valocidad y tiempos de cada estado
  float STOP_V = 0.0f;
  float STOP_W = 0.0f;
  float FORWARD_V = 0.3f;
  float FORWARD_W = 0.0f;
  float TURN_V = 0.0f;
  float TURN_W = 0.3f;
  float OBSTACLE_DISTANCE = 1.0f;
  const rclcpp::Duration TURN_SEC {2s};
  const rclcpp::Duration ROTATION_SEC {4s};
  const rclcpp::Duration LASER_DATA_SEC {1s};
  int obstacle_position_ = 0;

  // Publicadores y suscriptores para cada tipo de mensaje
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist out_vel_;
  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;
};
}  // namespace avoid_obstacle_al
#endif  // AVOID_OBSTACLE_AL__AVOIDOBSTACLEALNODE_HPP_