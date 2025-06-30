// Dependecias.
#include "rclcpp/rclcpp.hpp"

// Tipo de mensaje.
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Creación del nodo de nombre publisher_node.
  auto node = rclcpp::Node::make_shared("publisher_node");

  // Creación del publicador:
  // auto publisher = node->create_publisher<message_type>("topic",frecuencia);
  auto publisher = node->create_publisher<std_msgs::msg::Int32>("int_topic", 10);

  // Tipo de mensaje.
  std_msgs::msg::Int32 message;

  // Valor inicial = 0.
  message.data = 0;

  // Publica cada 500 ms e incrementa en 1 la variable message.data.
  rclcpp::Rate loop_rate(500ms);
  while (rclcpp::ok()) {
    message.data += 1;
    publisher->publish(message);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}