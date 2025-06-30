// Permite el acceso a la mayoría de tipos y funciones de C++.
#include "rclcpp/rclcpp.hpp"
int main(int argc, char * argv[]){

	// Extrae los argumentos con los que el proceso se ejecutó.
	rclcpp::init(argc,argv);
	
	// Creación de un nodo en ROS2 llamado simple_node.
	// Tipo del nodo: std::shared_ptr (alias).
	// make_shared: Método estático.
	// Otras alternativas:
	// 1. std::shared_ptr<rclcpp::Node> node = std::shared_ptr<rclcpp::Node> (new rclcpp::Node(“simple_node”));
	// 2. std::shared_ptr<rclcpp::Node> node = std::shared_ptr<rclcpp::Node> (“simple_node”);
	// 3. rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node> (“simple_node”);
	// 4. auto node = std::make_shared("simple_node");
	auto node = rclcpp::Node::make_shared("simple_node");
	
	// Bloquea la ejecución del programa para que el programa no termine.
	rclcpp::spin(node);
	
	// Gestiona el cierre de un nodo antes de acabar el programa.
	rclcpp::shutdown();
	
	return 0;
}
