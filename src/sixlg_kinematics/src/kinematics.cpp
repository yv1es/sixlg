
#include "rclcpp/rclcpp.hpp"
#include "sixlg_kinematics/Kinematics.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Kinematics>());
    rclcpp::shutdown();
    return 0;
}