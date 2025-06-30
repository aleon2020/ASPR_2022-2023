// Dependencias.
#include "rclcpp/rclcpp.hpp"

// Tipo de mensaje.
#include "std_msgs/msg/int32.hpp"

// Indican cuándo y por dónde entra un argumento.
using std::placeholders::_1;

class SubscriberNode : public rclcpp::Node
{
// Nombre del nodo suscriptor
public:
  SubscriberNode()
  : Node("subscriber_node")
  {
    // Creación del suscriptor:
    // publisher_ = create_publisher<message_type>("topic",frecuencia,callback);
    subscriber_ = create_subscription<std_msgs::msg::Int32>(
      "int_topic", 10,
      std::bind(&SubscriberNode::callback, this, _1));
  }

  // Formato del mensaje recibido por el suscriptor:
  // [INFO] [tiempo_recibido] [nombre_nodo]: Hello 1
  // [INFO] [tiempo_recibido] [nombre_nodo]: Hello 2
  // ...
  // [INFO] [tiempo_recibido] [nombre_nodo]: Hello N
  // Formato del mensaje recibido por el topic:
  // data: 1
  // ---
  // data: 2
  // ---
  // data: N
  void callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Hello %d", msg->data);
  }

private:
  // Constructor: Recibe un entero.
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;
};

// Función principal (método main).
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
