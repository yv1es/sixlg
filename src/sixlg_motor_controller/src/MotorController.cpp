
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
    : Node("motor_controller"),
      serialPort{mn::CppLinuxSerial::SerialPort("/dev/ttyACM0", mn::CppLinuxSerial::BaudRate::B_230400, mn::CppLinuxSerial::NumDataBits::EIGHT, mn::CppLinuxSerial::Parity::NONE, mn::CppLinuxSerial::NumStopBits::ONE)}
{
    subscription = create_subscription<sixlg_interfaces::msg::ServoAngles>(
        "topic", 10, [this](const sixlg_interfaces::msg::ServoAngles msg)
        { topic_callback(msg); });

    serialPort.Open();
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Arduino resets after opening serial connection

    const auto delay = std::chrono::milliseconds(500); 
    while (true)
    {
        moveLegXYZ(0.07, 0.13, -0.06); 
        std::this_thread::sleep_for(delay); 

        moveLegXYZ(0.0, 0.10, -0.01); 
        std::this_thread::sleep_for(delay); 

        moveLegXYZ(-0.07, 0.07, -0.06); 
        std::this_thread::sleep_for(delay); 
    }
}

void moveLegTrajectory()


void MotorController::moveLegXYZ(double X, double Y, double Z)
{
    double lF = 0.09; // length Femur
    double lT = 0.14; // length Tibia

    double lF_sqr = std::pow(lF, 2);
    double lT_sqr = std::pow(lF, 2);

    double L_sqr = std::pow(X, 2) + std::pow(Y, 2);
    double l_sqr = L_sqr + std::pow(Z, 2);
    double l = std::sqrt(l_sqr);

    double J1 = std::atan(X / Y) + M_PI / 2;

    double J2 = std::acos((lF_sqr + l_sqr - lT_sqr) / (2 * lF * l)) - std::atan(-Z / l) + M_PI/2;

    double J3 = std::acos((lF_sqr + lT_sqr - l_sqr) / (2 * lF * lT));

    RCLCPP_INFO(this->get_logger(), "Target XYZ = %f; %f; %f; => J1=%f   J2=%f   J3=%f   ", X, Y, Z, J1, J2, J3);
    writeServoAngles({J1, J2, J3}); 
}

void MotorController::topic_callback(const sixlg_interfaces::msg::ServoAngles msg)
{

    // writeServoAngles(msg.angles);
}

void MotorController::writeServoAngles(const std::array<double, SERVO_COUNT> angles)
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

    RCLCPP_INFO(this->get_logger(), "Sent %s to Arduino", strm.str().c_str());
}