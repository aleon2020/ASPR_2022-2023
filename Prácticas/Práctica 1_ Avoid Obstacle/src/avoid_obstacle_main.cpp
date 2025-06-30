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

#include "avoid_obstacle_forocoches/AvoidObstacleNode.hpp"
#include "avoid_obstacle_forocoches/DebugNode.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto mov_node = std::make_shared<AvoidObstacleNode>();
  auto debug_node = std::make_shared<DebugNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  // rclcpp::executors::MultiThreadedExecutor executor(
  //   rclcpp::executor::ExecutorArgs(), 8);

  executor.add_node(mov_node);
  executor.add_node(debug_node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
