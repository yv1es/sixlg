#pragma once

#include "rclcpp/rclcpp.hpp"

#include <Eigen/Dense>

#include <array>
#include <map> 
#include <stdint.h>


enum class Side { Left, Right}; 

// Constants
const double STRIDE_LENGTH = 0.1; // [m] 
const double lF = 0.08; // [m] length Femur
const double lT = 0.12; // [m] length Tibia
const Eigen::Vector3d restingPoint{0, 0.1, -0.1}; // [m]

class Leg
{
public:
    Leg(const uint32_t index, const Side side, const double angleToX);
    ~Leg() = default;

    Eigen::Vector3d computeJointStates(const double t) const;
    void computeTrajectory(const Eigen::Vector3d &walkingDirection); 

private:
    Eigen::Vector3d computeJointStatesFromTipPos(const Eigen::Vector3d &tipPos) const;
    Eigen::Vector3d quadraticBezier(const std::array<Eigen::Vector3d, 3> &points, const double t) const; 
    Eigen::Vector3d getRobotXDirection(const double angleToX) const;  // x direction of robot in legs coordinates

    const uint32_t m_index;
    const Side m_side; 
    const Eigen::Vector3d m_robotXdirection; 
    const double m_legXAngle; // angle between the robots x-Axis and legs x-Axis 
    std::function<Eigen::Vector3d(const double)> m_tipTrajectory; 
};