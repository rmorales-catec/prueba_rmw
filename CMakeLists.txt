cmake_minimum_required(VERSION 3.8)
project(prueba_rmw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)


add_executable(image_publisher_compressed src/image_publisher_compressed.cpp)
ament_target_dependencies(image_publisher_compressed rclcpp sensor_msgs cv_bridge OpenCV std_msgs)

add_executable(image_subscriber_compressed src/image_subscriber_compressed.cpp)
ament_target_dependencies(image_subscriber_compressed rclcpp sensor_msgs cv_bridge OpenCV std_msgs)

add_executable(image_publisher src/image_publisher_QoS.cpp)
ament_target_dependencies(image_publisher rclcpp sensor_msgs cv_bridge OpenCV std_msgs)

add_executable(image_subscriber src/image_subscriber_QoS.cpp)
ament_target_dependencies(image_subscriber rclcpp sensor_msgs cv_bridge OpenCV std_msgs)

install (TARGETS
  image_publisher
  image_subscriber
  image_publisher_compressed
  image_subscriber_compressed
  DESTINATION lib/${PROJECT_NAME}
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
