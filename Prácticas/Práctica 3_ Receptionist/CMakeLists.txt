cmake_minimum_required(VERSION 3.5)
project(recepcionist_forocoches)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(action_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sound_play REQUIRED)
find_package(dialogflow_ros2_interfaces REQUIRED)
find_package(gb_dialog REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)

find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    action_msgs
    vision_msgs
    rclcpp
    rclcpp_action
    behaviortree_cpp_v3
    ament_index_cpp
    nav2_msgs
    tf2_ros
    tf2_geometry_msgs
    geometry_msgs
    sound_play
    dialogflow_ros2_interfaces
    gb_dialog
    kobuki_ros_interfaces
)

include_directories(include ${ZMQ_INCLUDE_DIRS})

add_library(recepcionist_find_chair_bt_node SHARED src/recepcionist_forocoches/Find_Chair.cpp)
add_library(recepcionist_go_to_waypoint_bt_node SHARED src/recepcionist_forocoches/Go_to_Waypoint.cpp)
add_library(recepcionist_get_waypoint_bt_node SHARED src/recepcionist_forocoches/Get_Waypoint.cpp)
add_library(recepcionist_wait_person_bt_node SHARED src/recepcionist_forocoches/Wait_Person.cpp)
add_library(recepcionist_ask_person_bt_node SHARED src/recepcionist_forocoches/Ask_Name.cpp)
add_library(recepcionist_ask_drink_bt_node SHARED src/recepcionist_forocoches/Ask_Drink.cpp)
add_library(recepcionist_greet_bt_node SHARED src/recepcionist_forocoches/Greet.cpp)
add_library(recepcionist_order_drink_bt_node SHARED src/recepcionist_forocoches/Order_Drink.cpp)
add_library(recepcionist_tell_bt_node SHARED src/recepcionist_forocoches/Tell.cpp)
add_library(recepcionist_wait_drink_bt_node SHARED src/recepcionist_forocoches/Wait_Drink_Received.cpp)
add_library(recepcionist_deliver_drink_bt_node SHARED src/recepcionist_forocoches/Deliver_Drink.cpp)

list(APPEND plugin_libs
  recepcionist_find_chair_bt_node
  recepcionist_go_to_waypoint_bt_node
  recepcionist_get_waypoint_bt_node
  recepcionist_wait_person_bt_node
  recepcionist_ask_person_bt_node
  recepcionist_ask_drink_bt_node
  recepcionist_greet_bt_node
  recepcionist_order_drink_bt_node
  recepcionist_tell_bt_node
  recepcionist_wait_drink_bt_node
  recepcionist_deliver_drink_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(rec_forocoches src/recepcionist_main.cpp)
ament_target_dependencies(rec_forocoches ${dependencies})
target_link_libraries(rec_forocoches ${ZMQ_LIBRARIES})

install(TARGETS
  ${plugin_libs}
  rec_forocoches
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config launch behavior_tree_xml
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
