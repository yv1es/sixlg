#pragma once

#include "rclcpp/rclcpp.hpp"

#include "sixlg_interfaces/msg/servo_angles.hpp"

class MotorController : public rclcpp::Node
{
public:
    MotorController();

private:
    void topic_callback(const sixlg_interfaces::msg::ServoAngles msg) const;
    rclcpp::Subscription<sixlg_interfaces::msg::ServoAngles>::SharedPtr subscription;
};