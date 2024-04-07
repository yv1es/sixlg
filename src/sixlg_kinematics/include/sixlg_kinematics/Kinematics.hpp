#pragma once

#include "sixlg_interfaces/msg/servo_angles.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sixlg_kinematics/Leg.hpp"

class Kinematics : public rclcpp::Node
{
public:
    Kinematics();
    ~Kinematics() = default;

private:
    std::array<Leg, 6> m_legs;

    void makeStep();

    void timer_callback();
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<sixlg_interfaces::msg::ServoAngles>::SharedPtr m_publisher;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_subscription;

    double m_t;
};
