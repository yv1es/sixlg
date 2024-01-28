cmake_minimum_required(VERSION 3.8)
project(sixlg_motor_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sixlg_interfaces REQUIRED)


# building the library
add_library(MotorController src/MotorController.cpp)

# linking against my_interfaces
ament_target_dependencies(MotorController sixlg_interfaces rclcpp)

# find the header files
include_directories(include)

add_executable(motor_controller src/main.cpp)
ament_target_dependencies(motor_controller rclcpp)

target_link_libraries(motor_controller MotorController)



install(TARGETS
  motor_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
