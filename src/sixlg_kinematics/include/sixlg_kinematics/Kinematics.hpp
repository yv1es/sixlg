#pragma once

#include "sixlg_interfaces/msg/servo_angles.hpp"

#include "rclcpp/rclcpp.hpp"

#include "sixlg_kinematics/Leg.hpp"


class Kinematics : public rclcpp::Node
{
public:
    Kinematics();
    ~Kinematics() = default;

private:
    const std::array<Leg, 6> m_legs;

    void makeStep(); 

    void publishServoAnlges(std::array<_Float32, 18> &angles);
    void timer_callback();
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<sixlg_interfaces::msg::ServoAngles>::SharedPtr m_publisher;
};
