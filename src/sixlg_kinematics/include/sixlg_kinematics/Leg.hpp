#pragma once

#include "rclcpp/rclcpp.hpp"

#include <map> 
#include <stdint.h>
#include <vector>

using Vector3 = std::array<double, 3>; 

enum class InterpolationSchema { Linear, QuadraticBezier}; 
enum class Side { Left, Right}; 
enum class Stride { Forward }; 

// Constants
const double STRIDE_LENGTH = 0.1; // [m] 
const double lF = 0.08; // [m] length Femur
const double lT = 0.12; // [m] length Tibia


class Leg
{
public:
    Leg(uint32_t index, Side side, double angleToFront);
    ~Leg() = default;

    Vector3 computeJointStatesForward(const double t) const;

private:
    void computeTrajectories(); 

    Vector3 computeJointStatesFromXYZ(const Vector3 &xyz) const;

    const uint32_t m_index;
    const double  m_angleToFront;
    const Side m_side; 
    std::map<Stride, std::function<Vector3(double)>> m_strideTrajectories; 
    Vector3 quadraticBezier(const std::vector<Vector3> &points, const double t) const; 

};