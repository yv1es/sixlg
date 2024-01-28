
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "sixlg_interfaces/msg/servo_angles.hpp"

class TestPublisher : public rclcpp::Node
{
public:
    TestPublisher();

private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<sixlg_interfaces::msg::ServoAngles>::SharedPtr m_publisher;
    size_t m_count;
};
