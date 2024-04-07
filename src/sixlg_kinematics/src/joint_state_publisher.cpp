
#include "rclcpp/rclcpp.hpp"
#include "sixlg_kinematics/JointStatePublisher.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStatePublisher>());
    rclcpp::shutdown();
    return 0;
}