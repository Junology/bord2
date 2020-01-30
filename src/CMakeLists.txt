cmake_minimum_required(VERSION 3.0)

add_executable(
  ${PROJECT_NAME}
  main.cpp
  MainFrame.cpp
  PlTangView.cpp
  Figure3DView.cpp
  PlTangEntryDialog.cpp
  BordPreviewDialog.cpp
  WxGSScheme.cpp
  math/AffHypPlane.cpp
  math/QuadraticCurve.cpp
  figures/QBezierTriangle.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  Threads::Threads
  Eigen3::Eigen
  ${wxWidgets_LIBRARIES}
)

set_target_properties(
  ${PROJECT_NAME}
  PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)
