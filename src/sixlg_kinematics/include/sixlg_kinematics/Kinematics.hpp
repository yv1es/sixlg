#pragma once 


#include "rclcpp/rclcpp.hpp"

#include "sixlg_kinematics/Leg.hpp"



class Kinematics : public rclcpp::Node
{
public:
    Kinematics();
    ~Kinematics() = default; 

private:
    Leg m_leg; 
}; 
