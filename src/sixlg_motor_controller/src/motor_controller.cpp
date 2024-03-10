
#include "rclcpp/rclcpp.hpp"
#include "sixlg_motor_controller/MotorController.hpp"

#include <stdio.h>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorController>());
    rclcpp::shutdown();
    return 0;
}