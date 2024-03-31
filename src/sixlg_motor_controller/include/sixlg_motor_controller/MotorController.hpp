#pragma once

#include "rclcpp/rclcpp.hpp"

#include "sixlg_interfaces/msg/servo_angles.hpp"
#include "sixlg_motor_controller/SerialPort.hpp"

const size_t SERVO_COUNT = 18; 

class MotorController : public rclcpp::Node
{
public:
    MotorController();
    ~MotorController() = default; 

private:
    void servoAnglesCallback(const sixlg_interfaces::msg::ServoAngles msg);
    void writeServoAngles(const std::array<_Float32, SERVO_COUNT> angles); 

    rclcpp::Subscription<sixlg_interfaces::msg::ServoAngles>::SharedPtr m_subscription;
    mn::CppLinuxSerial::SerialPort m_serialPort; 
};