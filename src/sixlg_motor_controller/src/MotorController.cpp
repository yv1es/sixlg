
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
    (void)msg;
}
