// Dependencias.
#include "rclcpp/rclcpp.hpp"

// Tipo de mensaje.
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

// Indican cuándo y por dónde entra un argumento.
using std::placeholders::_1;

class PublisherNode : public rclcpp::Node
{
// Nombre del nodo publicador.
public:
  PublisherNode()
  : Node("publisher_node")
  {
    // Creación del publicador:
    // publisher_ = create_publisher<message_type>("topic",frecuencia);
    publisher_ = create_publisher<std_msgs::msg::Int32>("int_topic", 10);

    // Método create_wall_timer():
    // Establece las frecuencias mediante la creación de un contador.
    timer_ = create_wall_timer(500ms, std::bind(&PublisherNode::timer_callback, this));
  }

  // Publica cada 500 ms e incrementa en 1 la variable message.data.
  void timer_callback()
  {
    message_.data += 1;
    publisher_->publish(message_);
  }

private:
  // Constructor: Envía un entero.
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Tipo de mensaje.
  std_msgs::msg::Int32 message_;
};

// Función principal (método main).
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
