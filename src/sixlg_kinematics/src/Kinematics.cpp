
#include "sixlg_kinematics/Kinematics.hpp"

#include <chrono>
#include <functional>
#include <array>
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
          Leg(0),
          Leg(1),
          Leg(2),
          Leg(3),
          Leg(4),
          Leg(5),
      })
{
    m_publisher = create_publisher<sixlg_interfaces::msg::ServoAngles>("sixlg/servo_angles", 10);
    m_timer = create_wall_timer(
        std::chrono::milliseconds(1500), [this]()
        { timer_callback(); });
    RCLCPP_INFO(this->get_logger(), "Kinematics node up!");
}

void Kinematics::publishServoAnlges(std::array<_Float32, 18> &angles)
{
    RCLCPP_INFO(this->get_logger(), "Publishing servo angles");
    sixlg_interfaces::msg::ServoAngles msg;
    msg.angles = angles;
    m_publisher->publish(msg);
}

// For testing and development
void Kinematics::makeStep()
{
    RCLCPP_INFO(this->get_logger(), "Make step");
    sixlg_interfaces::msg::ServoAngles servoAngles;
    for (size_t i = 0; i < servoAngles.angles.size(); i++)
    {
        servoAngles.angles[i] = 0;
    }

    uint samples = 60;
    const auto delay = std::chrono::milliseconds(20);

    auto push = m_legs[0].computeJointStatesFromTrajectory({
                                                               {0.12, 0.1, -0.1},
                                                               {0.0, 0.1, -0.1},
                                                               {-0.12, 0.1, -0.1},
                                                           },
                                                           samples);

    auto pull = m_legs[0].computeJointStatesFromTrajectory({
                                                               {-0.12, 0.1, -0.1},
                                                               {0.0, 0.1, 0.05},
                                                               {0.12, 0.1, -0.1},
                                                           },
                                                           samples);

    // concat
    push.insert(push.end(), pull.begin(), pull.end());

    for (auto& angle : servoAngles.angles) {
        angle = M_PI/2; 
    } 
    m_publisher->publish(servoAngles);

    std::this_thread::sleep_for(delay);
}

void Kinematics::timer_callback()
{
    makeStep();
}
