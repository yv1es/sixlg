
#include "sixlg_kinematics/Leg.hpp"

#include <cmath>

namespace
{
    Vector3 add(Vector3 a, Vector3 b)
    {
        return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
    }

    Vector3 scale(double t, Vector3 a)
    {
        return Vector3{a[0] * t, a[1] * t, a[2] * t};
    }
}

Leg::Leg(uint32_t index, Side side, double angleToFront) : m_index(index),
                                                           m_angleToFront(angleToFront),
                                                           m_side(side)
{
    computeTrajectories(); 
}

void Leg::computeTrajectories()
{
    const auto forward = [this](double t) -> Vector3
    {
        assert(0 <= t && t <= 1);
        if (t < 0.5)
        {
            // push
            t = t * 2;
            return quadraticBezier({
                                       {STRIDE_LENGTH / 2, 0.1, -0.1},
                                       {0, 0.1, -0.1},
                                       {-STRIDE_LENGTH / 2, 0.1, -0.1},
                                   },
                                   t);
        }
        else
        {
            // pull
            t = (t - 0.5) * 2;
            return quadraticBezier({
                                       {-STRIDE_LENGTH / 2, 0.1, -0.1},
                                       {0, 0.1, 0.05},
                                       {STRIDE_LENGTH / 2, 0.1, -0.1},
                                   },
                                   t);
        }
    };
    m_strideTrajectories[Stride::Forward] = forward; 
}


Vector3 Leg::computeJointStatesForward(const double t) const {
    return computeJointStatesFromXYZ(m_strideTrajectories.at(Stride::Forward)(t)); 
}


/*
 * Inverse kinematics to transform leg tip point to joint states
 */
Vector3 Leg::computeJointStatesFromXYZ(const Vector3 &xyz) const
{
    double X = xyz[0];
    double Y = xyz[1];
    double Z = xyz[2];

    std::cout << "XYZ = " << X << ";" << Y << ";" << Z << ";\n"; 

    double lF_sqr = std::pow(lF, 2);
    double lT_sqr = std::pow(lT, 2);

    double L_sqr = std::pow(X, 2) + std::pow(Y, 2);
    double l_sqr = L_sqr + std::pow(Z, 2);
    double l = std::sqrt(l_sqr);

    double J0 = std::atan(X / Y) + M_PI / 2;
    double J1 = std::acos((lF_sqr + l_sqr - lT_sqr) / (2 * lF * l)) - std::atan(-Z / l) + M_PI / 2;
    double J2 = std::acos((lF_sqr + lT_sqr - l_sqr) / (2 * lF * lT));
    J2 = M_PI - J2;

    if (m_side == Side::Right)
    {
        J0 = M_PI - J0;
        J1 = M_PI - J1;
        J2 = M_PI - J2;
    }
    return {J0, J1, J2};
}


Vector3 Leg::quadraticBezier(const std::vector<Vector3> &points, const double t) const
{
    const auto curve = [&points](double t)
    {
        return add(scale(1 - t, (add(scale(1 - t, points[0]), scale(t, points[1])))), scale(t, add(scale(1 - t, points[1]), scale(t, points[2]))));
    };
    return curve(t);
}
