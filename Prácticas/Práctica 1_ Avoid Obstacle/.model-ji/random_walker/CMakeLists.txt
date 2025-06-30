cmake_minimum_required(VERSION 3.8)
project(random_walker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
  kobuki_ros_interfaces
  std_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/random_walker/RandomWalkerNode.cpp
  src/random_walker/DebugNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(wanderer
  src/random_walker_main.cpp
)
ament_target_dependencies(wanderer ${dependencies})
target_link_libraries(wanderer ${PROJECT_NAME})

install(TARGETS
  wanderer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
