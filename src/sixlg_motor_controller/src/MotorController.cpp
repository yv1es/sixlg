#include <chrono>
#include <fstream>
#include <iostream>
#include <cmath>
#include <sstream>
#include <stdint.h>
#include <string>
#include <thread>

#include "sixlg_motor_controller/MotorController.hpp"

MotorController::MotorController()
    : Node("sixlg_motor_controller"),
      serialPort{mn::CppLinuxSerial::SerialPort("/dev/ttyACM0", mn::CppLinuxSerial::BaudRate::B_115200, mn::CppLinuxSerial::NumDataBits::EIGHT, mn::CppLinuxSerial::Parity::NONE, mn::CppLinuxSerial::NumStopBits::ONE)}
{
    subscription = create_subscription<sixlg_interfaces::msg::ServoAngles>(
        "sixlg/servo_angles", 10, [this](const sixlg_interfaces::msg::ServoAngles msg)
        { servoAnglesCallback(msg); });

    serialPort.Open();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Arduino resets after opening serial connection
    RCLCPP_INFO(this->get_logger(), "MotorController is up!");
}

void MotorController::servoAnglesCallback(const sixlg_interfaces::msg::ServoAngles msg)
{
    writeServoAngles(msg.angles);
}

void MotorController::writeServoAngles(const std::array<_Float32, SERVO_COUNT> angles)
{
    std::ostringstream strm;
    strm << "<";

    for (const auto angle : angles)
    {
        int32_t deg = angle * 180 / M_PI;
        strm << deg << ",";
    }

    strm.seekp(-1, strm.cur);
    strm << ">";

    serialPort.Write(strm.str());

    RCLCPP_INFO(this->get_logger(), "Sending %s to Arduino", strm.str().c_str());
}