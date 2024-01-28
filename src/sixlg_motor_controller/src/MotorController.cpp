
#include <sstream>
#include <string> 

#include "sixlg_motor_controller/MotorController.hpp"

MotorController::MotorController()
    : Node("motor_controller")
{
    subscription = create_subscription<sixlg_interfaces::msg::ServoAngles>(
        "topic", 10, [this](const sixlg_interfaces::msg::ServoAngles msg)
        { topic_callback(msg); });
}

void MotorController::topic_callback(const sixlg_interfaces::msg::ServoAngles msg) const
{
    auto angles = msg.angles;
    RCLCPP_INFO(this->get_logger(), "Received %d", angles[0]);
}
