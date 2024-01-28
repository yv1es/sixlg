
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include "sixlg_motor_controller/MotorController.hpp"

MotorController::MotorController()
    : Node("motor_controller"),
      serialPort{mn::CppLinuxSerial::SerialPort("/dev/ttyACM0", mn::CppLinuxSerial::BaudRate::B_230400, mn::CppLinuxSerial::NumDataBits::EIGHT, mn::CppLinuxSerial::Parity::NONE, mn::CppLinuxSerial::NumStopBits::ONE)}
{
    subscription = create_subscription<sixlg_interfaces::msg::ServoAngles>(
        "topic", 10, [this](const sixlg_interfaces::msg::ServoAngles msg)
        { topic_callback(msg); });

    serialPort.Open();
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Arduino resets after opening serial connection
}

void MotorController::topic_callback(const sixlg_interfaces::msg::ServoAngles msg)
{
    writeServoAngles(msg.angles);
}

void MotorController::writeServoAngles(const std::array<int32_t, SERVO_COUNT> angles)
{
    std::ostringstream strm;
    strm << "<";
    for (const auto angle : angles)
    {
        strm << angle << ",";
    }
    strm.seekp(-1, strm.cur);
    strm << ">";

    serialPort.Write(strm.str());

    RCLCPP_INFO(this->get_logger(), "Sent %s to Arduino", strm.str().c_str());
}