#pragma once

#include "rclcpp/rclcpp.hpp"

#include "sixlg_interfaces/msg/servo_angles.hpp"
#include "sixlg_motor_controller/SerialPort.hpp"

const int SERVO_COUNT = 3; 

class MotorController : public rclcpp::Node
{
public:
    MotorController();
    ~MotorController() = default; 

private:
    void moveLegXYZ(double X, double Y, double Z); 
    void topic_callback(const sixlg_interfaces::msg::ServoAngles msg);
    void writeServoAngles(const std::array<double, SERVO_COUNT> angles); 

    rclcpp::Subscription<sixlg_interfaces::msg::ServoAngles>::SharedPtr subscription;
    mn::CppLinuxSerial::SerialPort serialPort; 
};