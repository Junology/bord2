## Set test project name
set(PROJECT_NAME_TEST ${PROJECT_NAME}-test)

## Test compile flags
set(CMAKE_CXX_FLAGS "-O0 -g ${CMAKE_CXX_FLAGS}")

## Build tests
#add_executable(
#  ${PROJECT_NAME_TEST}
#)

#target_link_libraries(
#  ${PROJECT_NAME_TEST}
#)

## Execute Test
#add_test(${PROJECT_NAME_TEST} AllTests COMMAND ${PROJECT_NAME_TEST})

