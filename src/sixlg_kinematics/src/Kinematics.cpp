
#include "sixlg_kinematics/Kinematics.hpp"

#include <chrono>
#include <functional>
#include <array>
#include <memory>
#include <string>

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
    RCLCPP_INFO(this->get_logger(), "Kinematics node up!");
    m_publisher = create_publisher<sixlg_interfaces::msg::ServoAngles>("sixlg/servo_angles", 10);
    m_timer = create_wall_timer(
        std::chrono::milliseconds(2000), [this]()
        { timer_callback(); });
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
    std::array<_Float32, 18> angles;
    servoAngles.angles = angles;
    for (auto& angle : angles)
    {
        angle = 0;
    }

    uint samples = 20;
    const auto delay = std::chrono::milliseconds(100);

    auto push = m_legs[0].computeJointStatesFromTrajectory({
                                                               {0.05, 0.12, -0.07},
                                                               {-0.05, 0.12, -0.07},
                                                           },
                                                           InterpolationSchema::Linear, samples);

    auto pull = m_legs[0].computeJointStatesFromTrajectory({
                                                               {-0.05, 0.12, -0.07},
                                                               {0.00, 0.12, -0.02},
                                                               {0.05, 0.12, -0.07},
                                                           },
                                                           InterpolationSchema::QuadraticBezier, samples);

    // concat
    push.insert(push.end(), pull.begin(), pull.end());

    for (const auto jointStates : push)
    {
        angles[0] = jointStates[0];
        angles[1] = jointStates[1];
        angles[2] = jointStates[2];
        m_publisher->publish(servoAngles);
        std::this_thread::sleep_for(delay);
    }
}

void Kinematics::timer_callback()
{
    makeStep();
}
