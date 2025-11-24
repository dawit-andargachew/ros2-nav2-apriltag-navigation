cmake_minimum_required(VERSION 3.8)
project(table_detection_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DetectTables.srv"
  DEPENDENCIES geometry_msgs
)

# --- Server node ---
add_executable(table_detection_server src/table_detection_server.cpp)
ament_target_dependencies(table_detection_server
  rclcpp sensor_msgs geometry_msgs tf2_ros tf2_geometry_msgs
)

# NEW recommended way:
rosidl_get_typesupport_target(cpp_typesupport_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(table_detection_server
  "${cpp_typesupport_server}")

# --- Client node ---
add_executable(table_detection_client src/table_detection_client.cpp)
ament_target_dependencies(table_detection_client rclcpp)

rosidl_get_typesupport_target(cpp_typesupport_client
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(table_detection_client
  "${cpp_typesupport_client}")


install(TARGETS
  table_detection_server
  table_detection_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
