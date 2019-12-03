cmake_minimum_required(VERSION 3.0)

file(GLOB PROJECT_CPP "*.cpp")

add_executable(
  ${PROJECT_NAME}
  ${PROJECT_CPP}
)

target_link_libraries(
  ${PROJECT_NAME}
  Threads::Threads
  ${wxWidgets_LIBRARIES}
)

set_target_properties(
  ${PROJECT_NAME}
  PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)
