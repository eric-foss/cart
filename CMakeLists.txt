cmake_minimum_required(VERSION 3.5)
project(cart)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(/usr/include)
link_directories(/usr/lib)

add_executable(motor src/motor.cpp)
add_executable(rotate src/rotate.cpp)
add_executable(threesixty src/threesixty.cpp)
add_executable(magnetometer_node src/magnetometer_node.cpp)
add_executable(incline src/incline.cpp)
add_executable(servo src/servo.cpp)
ament_target_dependencies(motor rclcpp std_msgs)
ament_target_dependencies(magnetometer_node rclcpp std_msgs)
ament_target_dependencies(rotate rclcpp std_msgs)
ament_target_dependencies(threesixty rclcpp std_msgs)
ament_target_dependencies(incline rclcpp std_msgs)
ament_target_dependencies(servo rclcpp std_msgs)

target_link_libraries(servo robotcontrol)
target_link_libraries(incline robotcontrol)
target_link_libraries(motor robotcontrol)
target_link_libraries(rotate robotcontrol)
target_link_libraries(magnetometer_node robotcontrol)
target_link_libraries(threesixty robotcontrol)

install(TARGETS
  magnetometer_node
  incline
  servo
  motor
  rotate
  threesixty
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
