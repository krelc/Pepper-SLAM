cmake_minimum_required(VERSION 3.8)
project(pepper_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)


add_executable(face_sub src/face_sub.cpp)
ament_target_dependencies(face_sub rclcpp sensor_msgs cv_bridge OpenCV image_transport)

add_executable(laser_data_transform src/laser_data_transform.cpp)
ament_target_dependencies(laser_data_transform rclcpp tf2_ros geometry_msgs sensor_msgs)

add_executable(pepper_teleop src/pepper_teleop.cpp)
ament_target_dependencies(pepper_teleop rclcpp geometry_msgs)

install(TARGETS
  face_sub
  laser_data_transform
  pepper_teleop
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)


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

ament_package()
