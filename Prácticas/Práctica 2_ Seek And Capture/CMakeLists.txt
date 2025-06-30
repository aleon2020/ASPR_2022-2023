cmake_minimum_required(VERSION 3.5)
project(seek_and_capture_forocoches)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    tf2_ros
    tf2_geometry_msgs
    behaviortree_cpp_v3
    sensor_msgs
    vision_msgs
    geometry_msgs
    ament_index_cpp
    kobuki_ros_interfaces
)

include_directories(include ${ZMQ_INCLUDE_DIRS})

add_library(seek_and_capture_navigation_bt_node SHARED src/seek_and_capture_forocoches/Navigation.cpp
src/seek_and_capture_forocoches/PIDController.cpp src/seek_and_capture_forocoches/DebugNode.cpp)
add_library(seek_and_capture_find_person_bt_node SHARED src/seek_and_capture_forocoches/FindPerson.cpp
src/seek_and_capture_forocoches/DebugNode.cpp)
add_library(seek_and_capture_is_person_bt_node SHARED src/seek_and_capture_forocoches/IsPerson.cpp
src/seek_and_capture_forocoches/DebugNode.cpp)
add_library(seek_and_capture_turn_bt_node SHARED src/seek_and_capture_forocoches/Turn.cpp
src/seek_and_capture_forocoches/DebugNode.cpp)

list(APPEND plugin_libs
  seek_and_capture_navigation_bt_node
  seek_and_capture_find_person_bt_node
  seek_and_capture_is_person_bt_node
  seek_and_capture_turn_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(sac_forocoches src/seek_and_capture_main.cpp)
ament_target_dependencies(sac_forocoches ${dependencies})
target_link_libraries(sac_forocoches ${ZMQ_LIBRARIES})

install(TARGETS
  ${plugin_libs}
  sac_forocoches
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch behavior_tree_xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
