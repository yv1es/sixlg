
#pragma once

#include "sixlg_interfaces/msg/servo_angles.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"

#include <array>
#include <string>

class JointStatePublisher : public rclcpp::Node
{
public:
    JointStatePublisher();
    ~JointStatePublisher() = default;

private:
    void publishJointStates(const sixlg_interfaces::msg::ServoAngles msg);

    rclcpp::Subscription<sixlg_interfaces::msg::ServoAngles>::SharedPtr m_subscription;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_publisher;
    std::array<std::string, 18> m_jointNames; 
};