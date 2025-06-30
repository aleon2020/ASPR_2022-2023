cmake_minimum_required(VERSION 3.5)
project(clase_cmake)

include_directories(include)

add_executable(program 
  src/program.cpp
  src/functions.cpp
)
