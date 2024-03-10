
#include "sixlg_kinematics/Leg.hpp"

#include <cmath>

Leg::Leg(const uint32_t index) : m_index(index){};

Vector3 Leg::computeJointStatesFromXYZ(Vector3 xyz)
{
    double X = xyz[0];
    double Y = xyz[0];
    double Z = xyz[0];

    double lF_sqr = std::pow(m_lF, 2);

    double lT_sqr = std::pow(m_lF, 2);

    double L_sqr = std::pow(X, 2) + std::pow(Y, 2);
    double l_sqr = L_sqr + std::pow(Z, 2);
    double l = std::sqrt(l_sqr);

    double J1 = std::atan(X / Y) + M_PI / 2;

    double J2 = std::acos((lF_sqr + l_sqr - lT_sqr) / (2 * m_lF * l)) - std::atan(-Z / l) + M_PI / 2;

    double J3 = std::acos((lF_sqr + lT_sqr - l_sqr) / (2 * m_lF * m_lT));

    return {J1, J2, J3};
}

std::vector<Vector3> Leg::computeJointStatesFromTrajectory(const std::vector<Vector3> &points, const InterpolationSchema interpolationSchema, const uint32_t samples) const
{
    return {{0, 0, 0}}; 
}