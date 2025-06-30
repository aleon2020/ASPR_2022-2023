cmake_minimum_required(VERSION 3.5)
project(clase_cmake_program)

find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  ${clase_cmake_functions_INCLUDE_DIRS}
)

add_executable(program  src/program.cpp)
target_link_libraries(program 
  ${clase_cmake_functions_LIBS}
)
