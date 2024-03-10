#pragma once

#include <stdint.h>

#include <vector>

#include "rclcpp/rclcpp.hpp"


using Vector3 = std::array<double, 3>; 
enum class InterpolationSchema { Linear, QuadraticBezier}; 


class Leg
{
public:
    Leg(uint32_t index);
    ~Leg() = default;

    std::vector<Vector3> computeJointStatesFromTrajectory(const std::vector<Vector3>& points, const uint32_t samples) const; 

private:
    Vector3 computeJointStatesFromXYZ(Vector3 xyz) const;

    const uint32_t m_index;

    const double m_lF = 0.09; // length Femur
    const double m_lT = 0.14; // length Tibia
};