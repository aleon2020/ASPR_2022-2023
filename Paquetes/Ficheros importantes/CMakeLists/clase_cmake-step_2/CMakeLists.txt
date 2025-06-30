cmake_minimum_required(VERSION 3.5)
project(clase_cmake)

include_directories(include)

add_library(functions_clase_cmake SHARED src/clase_cmake/functions.cpp)

add_executable(program  src/program.cpp)
target_link_libraries(program functions_clase_cmake)
