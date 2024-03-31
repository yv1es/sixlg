
#include "sixlg_kinematics/Leg.hpp"

#include <cmath>


namespace {
    Vector3 add(Vector3 a, Vector3 b) {
        return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; 
    }

    Vector3 scale(double t, Vector3 a) {
        return Vector3{a[0]*t, a[1]*t, a[2]*t}; 
    }
}



Leg::Leg(const uint32_t index) : m_index(index){};

Vector3 Leg::computeJointStatesFromXYZ(Vector3 xyz) const
{
    double X = xyz[0];
    double Y = xyz[1];
    double Z = xyz[2];

    double lF_sqr = std::pow(m_lF, 2);
    double lT_sqr = std::pow(m_lT, 2);

    double L_sqr = std::pow(X, 2) + std::pow(Y, 2);
    double l_sqr = L_sqr + std::pow(Z, 2);
    double l = std::sqrt(l_sqr);

    double J0 = std::atan(X / Y) + M_PI / 2;
    double J1 = std::acos((lF_sqr + l_sqr - lT_sqr) / (2 * m_lF * l)) - std::atan(-Z / l) + M_PI / 2;
    double J2 = std::acos((lF_sqr + lT_sqr - l_sqr) / (2 * m_lF * m_lT));
    J2 = M_PI - J2; 

    return {J0, J1, J2};
}

std::vector<Vector3> Leg::computeJointStatesFromTrajectory(const std::vector<Vector3> &points,  const uint32_t samples) const
{
    std::vector<Vector3> ret;     

    const auto curve = [&points](double t){   
        return add(scale(1-t, ( add(  scale(1-t, points[0])  , scale(t, points[1])))) , scale(t,add( scale(1-t, points[1]) , scale(t, points[2])))); 
    }; 

    for (double i = 0; i < samples; i++) {
        double t = i / samples; 
        auto s = curve(t); 
        std::cout << "computed sample: " << s[0] << "; " << s[1] << "; " << s[2] << "; " << std::endl; 
        ret.push_back(computeJointStatesFromXYZ(s)); 
    }

    return ret; 
}