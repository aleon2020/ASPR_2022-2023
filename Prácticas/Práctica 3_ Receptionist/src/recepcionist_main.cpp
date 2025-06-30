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

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("recepcionist_forocoches_node");

  node->declare_parameter("door.pose.position.x", 0.0);
  node->declare_parameter("door.pose.position.y", 0.0);
  node->declare_parameter("door.pose.orientation.w", 0.0);
  node->declare_parameter("party.pose.position.x", 0.0);
  node->declare_parameter("party.pose.position.y", 0.0);
  node->declare_parameter("party.pose.orientation.w", 0.0);
  node->declare_parameter("bar.pose.position.x", 0.0);
  node->declare_parameter("bar.pose.position.y", 0.0);
  node->declare_parameter("bar.pose.orientation.w", 0.0);
  node->declare_parameter("navigation.tolerance", 0.35);

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("recepcionist_find_chair_bt_node"));
  factory.registerFromPlugin(loader.getOSName("recepcionist_go_to_waypoint_bt_node"));
  factory.registerFromPlugin(loader.getOSName("recepcionist_get_waypoint_bt_node"));
  factory.registerFromPlugin(loader.getOSName("recepcionist_wait_person_bt_node"));
  factory.registerFromPlugin(loader.getOSName("recepcionist_ask_person_bt_node"));
  factory.registerFromPlugin(loader.getOSName("recepcionist_ask_drink_bt_node"));
  factory.registerFromPlugin(loader.getOSName("recepcionist_greet_bt_node"));
  factory.registerFromPlugin(loader.getOSName("recepcionist_order_drink_bt_node"));
  // factory.registerFromPlugin(loader.getOSName("recepcionist_tell_bt_node"));
  factory.registerFromPlugin(loader.getOSName("recepcionist_wait_drink_bt_node"));
  factory.registerFromPlugin(loader.getOSName("recepcionist_deliver_drink_bt_node"));

  std::string pkgpath = ament_index_cpp::get_package_share_directory("recepcionist_forocoches");
  std::string xml_file = pkgpath + "/behavior_tree_xml/recepcionist.xml";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 1666, 1667);

  rclcpp::Rate rate(10);

  bool finish = false;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
