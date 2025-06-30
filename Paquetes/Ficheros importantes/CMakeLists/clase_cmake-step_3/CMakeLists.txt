cmake_minimum_required(VERSION 3.5)
project(clase_cmake)

find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_library(functions_clase_cmake SHARED src/clase_cmake/functions.cpp)

add_executable(program  src/program.cpp)
target_link_libraries(program 
  functions_clase_cmake
  ${OpenCV_LIBS}
)
