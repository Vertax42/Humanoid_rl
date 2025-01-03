#ifndef BODY_CONTROL_H
#define BODY_CONTROL_H
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>

constexpr double PI = 3.14159265358979323846;
constexpr int CONTROL_FREQUENCY = 100;
constexpr int N_JOINTS = 30;
constexpr int N_HAND_JOINTS = 18; // left arm(7) + neck(2) + right arm(7) + waist(2)
constexpr int N_LEG_JOINTS = 12;  // left leg(6) + right leg(6)
constexpr std::array<double, 3> gravity = { 0.0, 0.0, -1.0 }; // gravity vector


std_msgs::Float64MultiArray BodyJointCommandWraper(std::vector<double> &pos_des, std::vector<double> &vel_des, std::vector<double> &kp, std::vector<double> &kd, std::vector<double> &torque);

#endif // BODY_CONTROL_H