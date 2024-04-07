
#include "sixlg_kinematics/JointStatePublisher.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"

#include <sstream>

JointStatePublisher::JointStatePublisher()
    : Node("sixlg_joint_state_publisher")
{
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            std::stringstream ss;
            ss << "leg" << i << "/j" << j << "_joint";
            m_jointNames.at(i * 3 + j) = ss.str();
        }
    }

    m_publisher = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    m_subscription = create_subscription<sixlg_interfaces::msg::ServoAngles>(
        "sixlg/servo_angles", 10, [this](const sixlg_interfaces::msg::ServoAngles msg)
        { publishJointStates(msg); });

    RCLCPP_INFO(this->get_logger(), "JointStatePublisher node up!");
}

void JointStatePublisher::publishJointStates(const sixlg_interfaces::msg::ServoAngles msg)
{
    sensor_msgs::msg::JointState jointState;
    jointState.header.stamp = get_clock()->now();

    for (int i = 0; i < 18; i++)
    {
        jointState.name.push_back(m_jointNames.at(i));
        jointState.position.push_back(msg.angles.at(i));
        jointState.velocity.push_back(0);
        jointState.effort.push_back(0);
    }
    m_publisher->publish(jointState);
}