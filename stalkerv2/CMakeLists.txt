cmake_minimum_required(VERSION 3.8)
project(stalkerv2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  vision_msgs
  tf2_ros
  tf2_geometry_msgs
  lifecycle_msgs
  # OpenCV
  # sensor_msgs
  # image_geometry
  # cv_bridge
  # image_transport
  # depth_image_proc
  # sensor_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/stalkerv2/PersonFollower.cpp
  src/stalkerv2/PIDController.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(follower src/main.cpp)
ament_target_dependencies(follower ${dependencies})
target_link_libraries(follower ${PROJECT_NAME})

install(DIRECTORY config launch models DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  follower
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
