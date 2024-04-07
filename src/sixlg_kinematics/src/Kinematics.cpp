
#include "sixlg_kinematics/Kinematics.hpp"

#include <chrono>
#include <functional>
#include <array>
#include <cmath>
#include <memory>
#include <string>

/*
    5-----0
   /       \
  4         1
   \       /
    3-----2
*/

Kinematics::Kinematics()
    : Node("sixlg_kinematics"),
      m_legs({
          Leg(0, Side::Right, -M_PI / 4),
          Leg(1, Side::Right, 0),
          Leg(2, Side::Right, M_PI / 4),
          Leg(3, Side::Left, M_PI / 4),
          Leg(4, Side::Left, 0),
          Leg(5, Side::Left, -M_PI / 4),
      }),
      m_t(0)
{

    m_subscription = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist msg)
        { 
            const Eigen::Vector3d direction{msg.linear.x, msg.linear.y, msg.linear.z}; 
            for (auto& leg : m_legs) {
               leg.computeTrajectory(direction); 
            } });

    m_publisher = create_publisher<sixlg_interfaces::msg::ServoAngles>("sixlg/servo_angles", 10);
    m_timer = create_wall_timer(
        std::chrono::milliseconds(20), [this]()
        { timer_callback(); });
    RCLCPP_INFO(this->get_logger(), "Kinematics node up!");
}

// For testing and development
void Kinematics::makeStep()
{
    sixlg_interfaces::msg::ServoAngles servoAngles;

    for (uint i = 0; i < m_legs.size(); i++)
    {
        double t = 0;
        if (i % 2 == 0)
        {
            t = m_t;
        }
        else
        {
            t = std::fmod(m_t + 0.5, 1);
        }

        const Eigen::Vector3d jointStates = m_legs.at(i).computeJointStates(t);
        servoAngles.angles[i * 3 + 0] = jointStates[0];
        servoAngles.angles[i * 3 + 1] = jointStates[1];
        servoAngles.angles[i * 3 + 2] = jointStates[2];
    }

    m_publisher->publish(servoAngles);

    // step t
    m_t = std::fmod(m_t + 0.01, 1);
}

void Kinematics::timer_callback()
{
    makeStep();
}
