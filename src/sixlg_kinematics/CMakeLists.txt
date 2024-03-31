cmake_minimum_required(VERSION 3.8)
project(sixlg_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sixlg_interfaces REQUIRED)

include_directories(include)



# Kinematics node
add_library(Leg src/Leg.cpp)
ament_target_dependencies(Leg sixlg_interfaces rclcpp )
add_library(Kinematics src/Kinematics.cpp)
target_link_libraries(Kinematics Leg)
ament_target_dependencies(Kinematics sixlg_interfaces rclcpp )


add_executable(kinematics src/kinematics.cpp)
ament_target_dependencies(kinematics rclcpp)
target_link_libraries(kinematics Kinematics)



# Kinematics node
add_library(JointStatePublisher src/JointStatePublisher.cpp)
ament_target_dependencies(JointStatePublisher sixlg_interfaces sensor_msgs rclcpp)

add_executable(joint_state_publisher src/joint_state_publisher.cpp)
ament_target_dependencies(joint_state_publisher rclcpp)
target_link_libraries(joint_state_publisher JointStatePublisher)


install(TARGETS
  kinematics
  joint_state_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()