cmake_minimum_required(VERSION 3.8)
project(l1w_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Threads REQUIRED)

# zsibot SDK (genisom_L1_sdk), installed via `sudo make install` (default prefix /usr/local).
# Override with -DZSIBOT_SDK_ROOT=<prefix> or the ZSIBOT_SDK_ROOT env var.
set(ZSIBOT_SDK_ROOT "$ENV{ZSIBOT_SDK_ROOT}" CACHE PATH "Install prefix of the zsibot SDK")

find_path(ZSIBOT_INCLUDE_DIR
  NAMES zsibot_sdk/zsibot_api.h
  HINTS ${ZSIBOT_SDK_ROOT}/include
  PATHS /usr/local/include /usr/include)

find_library(ZSIBOT_LIBRARY
  NAMES libzsibot.a zsibot
  HINTS ${ZSIBOT_SDK_ROOT}/lib
  PATHS /usr/local/lib /usr/lib)

if(NOT ZSIBOT_INCLUDE_DIR OR NOT ZSIBOT_LIBRARY)
  message(FATAL_ERROR
    "zsibot SDK not found (include=${ZSIBOT_INCLUDE_DIR}, lib=${ZSIBOT_LIBRARY}).\n"
    "Install genisom_L1_sdk (cmake .. && make && sudo make install) or pass -DZSIBOT_SDK_ROOT=<prefix>.")
endif()
message(STATUS "zsibot SDK: ${ZSIBOT_LIBRARY}")

add_library(zsibot_sdk STATIC IMPORTED)
set_target_properties(zsibot_sdk PROPERTIES
  IMPORTED_LOCATION "${ZSIBOT_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${ZSIBOT_INCLUDE_DIR}"
  INTERFACE_LINK_LIBRARIES Threads::Threads)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetWifi.srv"
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(l1w_driver_node src/l1w_driver_node.cpp)
target_include_directories(l1w_driver_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(l1w_driver_node zsibot_sdk "${cpp_typesupport_target}")
ament_target_dependencies(l1w_driver_node
  rclcpp std_msgs std_srvs geometry_msgs nav_msgs sensor_msgs diagnostic_msgs)

install(TARGETS l1w_driver_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
