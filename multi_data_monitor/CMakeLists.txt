cmake_minimum_required(VERSION 3.14)
project(multi_data_monitor)

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

find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_compile_definitions(FMT_HEADER_ONLY)

ament_auto_add_executable(loader
  src/loader.cpp
  src/core/config/parser.hpp
  src/core/config/parser.cpp
  src/core/config/types.hpp
  src/core/config/types.cpp
  src/core/config/utils.hpp
  src/core/config/utils.cpp
)
target_include_directories(loader PRIVATE src/core)

# ament_auto_add_library(${PROJECT_NAME} SHARED)
# target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})

# ament_auto_add_executable(debug)
# target_link_libraries(debug ${PROJECT_NAME})

pluginlib_export_plugin_description_file(rviz_common plugins/rviz.xml)
pluginlib_export_plugin_description_file(multi_data_monitor plugins/builtins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS
  scripts/tutorial
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE css icons plugins tutorials)
