
#include "sixlg_kinematics/Leg.hpp"

#include <cmath>

Leg::Leg(const uint index, const Side side, const double angleToX) : m_index(index),
                                                                     m_side(side),
                                                                     m_robotXdirection(getRobotXDirection(angleToX)),
                                                                     m_legXAngle(angleToX)
{
    computeTrajectory({-1, 0, 0}); 
}


Eigen::Vector3d Leg::computeJointStates(const double t) const {
    return computeJointStatesFromTipPos(m_tipTrajectory(t)); 
}

/*
 * Walking direction in robots coordinate system (base_link)
 */
void Leg::computeTrajectory(const Eigen::Vector3d &walkingDirection)
{
    Eigen::Vector3d step = STRIDE_LENGTH * walkingDirection.normalized();
    if (m_side == Side::Right) {
        step[1] *= -1; // mirror y Axis
    }

    // rotation around Z-axis
    Eigen::Matrix3d rotationMatrix;
    rotationMatrix << cos(-m_legXAngle), -sin(-m_legXAngle), 0,
        sin(-m_legXAngle), cos(-m_legXAngle), 0,
        0, 0, 1;

    step = rotationMatrix * step;

    Eigen::Vector3d front = 0.5 * step + restingPoint;
    Eigen::Vector3d middle = restingPoint;
    Eigen::Vector3d back = -0.5 * step + restingPoint;
    Eigen::Vector3d arch = restingPoint + Eigen::Vector3d{0, 0, 0.15};

    m_tipTrajectory = [this, front, middle, back, arch](double t) -> Eigen::Vector3d
    {
        assert(0 <= t && t <= 1);
        if (t < 0.5)
        {
            // push
            t = t * 2;
            return quadraticBezier({front, middle, back}, t);
        }
        else
        {
            // pull
            t = (t - 0.5) * 2;
            return quadraticBezier({back, arch, front}, t);
        }
    };
}

Eigen::Vector3d Leg::getRobotXDirection(const double angleToX) const
{
    // rotation around Z-axis
    Eigen::Matrix3d rotationMatrix;
    rotationMatrix << cos(angleToX), -sin(angleToX), 0,
        sin(angleToX), cos(angleToX), 0,
        0, 0, 1;

    return rotationMatrix * Eigen::Vector3d(1, 0, 0);
}

/*
 * Inverse kinematics to transform leg tip goal position to joint states
 */
Eigen::Vector3d Leg::computeJointStatesFromTipPos(const Eigen::Vector3d &tipPos) const
{
    const double X = tipPos[0];
    const double Y = tipPos[1];
    const double Z = tipPos[2];

    const double lF_sqr = std::pow(lF, 2);
    const double lT_sqr = std::pow(lT, 2);

    const double L_sqr = std::pow(X, 2) + std::pow(Y, 2);
    const double l_sqr = L_sqr + std::pow(Z, 2);
    const double l = std::sqrt(l_sqr);

    double J0 = std::atan(X / Y) + M_PI / 2;
    double J1 = std::acos((lF_sqr + l_sqr - lT_sqr) / (2 * lF * l)) - std::atan(-Z / l) + M_PI / 2;
    double J2 = M_PI - std::acos((lF_sqr + lT_sqr - l_sqr) / (2 * lF * lT));

    Eigen::Vector3d jointStates{J0, J1, J2};

    switch (m_side)
    {
    case Side::Right:
        return Eigen::Vector3d::Constant(M_PI) - jointStates;
    case Side::Left:
        return jointStates;
    default:
        std::cerr << "Unknown side" << std::endl;
        assert(false);
        return jointStates; // dummy return 
    }
}

/*
 * Quadratic Bezier interpolation
 */
Eigen::Vector3d Leg::quadraticBezier(const std::array<Eigen::Vector3d, 3> &points, const double t) const
{
    return (1 - t) * ((1 - t) * points[0] + t * points[1]) + t * ((1 - t) * points[1] + t * points[2]);
}
