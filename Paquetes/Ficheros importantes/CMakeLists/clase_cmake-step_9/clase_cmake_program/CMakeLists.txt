cmake_minimum_required(VERSION 3.5)
project(clase_cmake_program)

find_package(clase_cmake_functions REQUIRED)

set(dependencies
  clase_cmake_functions
)

add_executable(program src/program.cpp)
ament_target_dependencies(program ${dependencies})

install(TARGETS
    program
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
