cmake_minimum_required(VERSION 3.5)
project(clase_cmake_program)

include_directories(
  ${clase_cmake_functions_INCLUDE_DIRS}
)

add_executable(program  src/program.cpp)
target_link_libraries(program 
  ${clase_cmake_functions_LIBS}
)

install(TARGETS
    program
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)