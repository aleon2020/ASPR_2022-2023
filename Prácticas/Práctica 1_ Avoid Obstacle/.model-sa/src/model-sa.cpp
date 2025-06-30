#include "rclcpp/rclcpp.hpp"
#include "model-sa/base_controller.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<BaseController>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}