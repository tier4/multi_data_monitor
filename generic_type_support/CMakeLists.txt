cmake_minimum_required(VERSION 3.5)
project(generic_type_support)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic) # -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/message.cpp
  src/access.cpp
  src/typesupport.cpp
)

ament_auto_add_executable(debug
  src/debug.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(${PROJECT_NAME}_test test/test.cpp)
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp)
  target_include_directories(${PROJECT_NAME}_test PRIVATE include)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

ament_auto_package()
