cmake_minimum_required(VERSION 3.5)
project(clase_cmake_functions)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  OpenCV
  std_msgs
)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED src/clase_cmake_functions/functions.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

install(TARGETS
    ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
  
install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
