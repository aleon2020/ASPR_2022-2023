cmake_minimum_required(VERSION 3.8)
project(my_package)

# Se especifican los paquetes necesarios.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
    rclcpp
    std_msgs
)

# Compila indicando el nombre del resultado y sus fuentes.
add_executable(simple src/simple.cpp)
# Hace que los encabezados y las bibliotecas de otros paquetes 
# sean accesibles para el objetivo actual.
ament_target_dependencies(simple ${dependencies})

# Instala todos los programas y librerías del paquete 
# (tiene todos los ejecutables).
install(TARGETS
  simple
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()
