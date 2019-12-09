## Set test project name
set(PROJECT_NAME_TEST ${PROJECT_NAME}-test)

include_directories(${CMAKE_SOURCE_DIR}/src)

find_package(GTest)

if(GTEST_FOUND)
  enable_testing()
  include_directories(${GTEST_INCLUDE_DIRS})

  add_executable(${PROJECT_NAME_TEST}
	test_math.cpp)

  target_compile_options(${PROJECT_NAME_TEST} PUBLIC -O0 -g -Wall)

  target_link_libraries(${PROJECT_NAME_TEST}
	Threads::Threads
	Eigen3::Eigen
	GTest::GTest
	GTest::Main)

  add_test(${PROJECT_NAME_TEST} AllTests COMMAND ${PROJECT_NAME_TEST})
else(GTEST_FOUND)
  message("Google Test not found; tests are not enabled.")
endif(GTEST_FOUND)
