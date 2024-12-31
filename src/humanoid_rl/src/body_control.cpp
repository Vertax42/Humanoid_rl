#include "body_control.h"


std_msgs::Float64MultiArray BodyJointCommandWraper(std::vector<double> &pos_des, std::vector<double> &vel_des,
                                                   std::vector<double> &kp, std::vector<double> &kd,
                                                   std::vector<double> &torque)
{
    std_msgs::Float64MultiArray msg;
    // set layout
    msg.layout.dim.resize(2);
    msg.layout.dim[0].label = "parameters";
    msg.layout.dim[0].size = 5; // 5 种参数：pos_des, vel_des, kp, kd, torque
    msg.layout.dim[0].stride = N_JOINTS * 5;
    msg.layout.dim[1].label = "joints";
    msg.layout.dim[1].size = N_JOINTS;
    msg.layout.dim[1].stride = 1;
    msg.layout.data_offset = 0;

    // fill in data
    msg.data.insert(msg.data.end(), pos_des.begin(), pos_des.end());
    msg.data.insert(msg.data.end(), vel_des.begin(), vel_des.end());
    msg.data.insert(msg.data.end(), kp.begin(), kp.end());
    msg.data.insert(msg.data.end(), kd.begin(), kd.end());
    msg.data.insert(msg.data.end(), torque.begin(), torque.end());

    // wraping msg
    return msg;
}