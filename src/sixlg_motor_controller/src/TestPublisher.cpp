

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "sixlg_motor_controller/TestPublisher.hpp"


TestPublisher::TestPublisher()
    : Node("sixlg_test_publisher"), m_count(0)
{
    m_publisher = create_publisher<sixlg_interfaces::msg::ServoAngles>("topic", 10);
    m_timer = create_wall_timer(
        std::chrono::microseconds(500), [this]()
        { timer_callback(); });
}

void TestPublisher::timer_callback()
{
    sixlg_interfaces::msg::ServoAngles msg;

    std::array<int32_t, 3> angles;

    angles[0] = m_count % 180;
    angles[1] = 180 - m_count*2 % 180; 
    angles[2] = m_count/2 % 180; 

    m_count += 10; 

    msg.angles = angles;

    RCLCPP_INFO(this->get_logger(), "Publishing...");
    m_publisher->publish(msg);
}