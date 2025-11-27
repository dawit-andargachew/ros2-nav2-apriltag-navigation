cmake_minimum_required(VERSION 3.8)
project(group_6_assignment_1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DetectTables.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(table_detection_server src/TableDetectionServer.cpp)
ament_target_dependencies(table_detection_server
  rclcpp sensor_msgs geometry_msgs tf2_ros tf2_geometry_msgs
)

add_executable(goal_detector src/GoalDetector.cpp)
ament_target_dependencies(
  goal_detector
  rclcpp 
  tf2_ros
  tf2_geometry_msgs 
  geometry_msgs 
  apriltag_msgs
)

add_executable(initial_pose_pub src/InitialPosePub.cpp)
ament_target_dependencies(
  initial_pose_pub 
  rclcpp 
  geometry_msgs
  )

add_executable(navigation_orchestrator src/NavigationOrchestrator.cpp)
ament_target_dependencies(
  navigation_orchestrator 
  rclcpp 
  rclcpp_action
  geometry_msgs
  nav2_msgs
  lifecycle_msgs
  )

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

target_link_libraries(table_detection_server
  ${cpp_typesupport_target}
)

target_link_libraries(navigation_orchestrator
  ${cpp_typesupport_target}
)

#executable
install(TARGETS
  table_detection_server
  goal_detector
  initial_pose_pub
  navigation_orchestrator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
