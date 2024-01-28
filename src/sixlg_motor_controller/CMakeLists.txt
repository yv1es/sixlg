cmake_minimum_required(VERSION 3.8)
project(sixlg_motor_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sixlg_interfaces REQUIRED)

include_directories(include)

# building motor_controller
add_library(SerialPort src/SerialPort.cpp)
add_library(MotorController src/MotorController.cpp)
target_link_libraries(MotorController SerialPort)
ament_target_dependencies(MotorController sixlg_interfaces rclcpp)

add_executable(motor_controller src/motor_controller.cpp)
ament_target_dependencies(motor_controller rclcpp)
target_link_libraries(motor_controller SerialPort MotorController)


# building test_publisher
add_library(TestPublisher src/TestPublisher.cpp)
ament_target_dependencies(TestPublisher sixlg_interfaces rclcpp)

add_executable(test_publisher src/test_publisher.cpp)
ament_target_dependencies(test_publisher rclcpp)
target_link_libraries(test_publisher TestPublisher)


install(TARGETS
  test_publisher
  motor_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
