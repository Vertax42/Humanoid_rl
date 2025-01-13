#ifndef HUMANOID_RL_INFERENCE_H
#define HUMANOID_RL_INFERENCE_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <array>
#include <std_msgs/Float64MultiArray.h>
#include "humanoid_policy.h"


struct JointConfig {
    int id;
    std::string name;
    double pos_des;
    double vel_des;
    double kp;
    double kd;
    double torque;
    double upper_limit;
    double lower_limit;
};

struct ModelConfig {
    std::string model_path;
    int obs_dim;
    int action_dim;
    int history_length;
    int obs_history_length;
    double lin_sensitivity;
    double ang_sensitivity;
};

struct Quaternion {
    double w, x, y, z;
};

struct EulerAngle {
    double roll, pitch, yaw;
};

template <typename T>
T clip(T value, T min, T max);

template <typename T>
bool getParamHelper(ros::NodeHandle &nh, const std::string &param_name, T &param_value);

bool loadJointConfigs(ros::NodeHandle &nh, const std::string &param_name, std::vector<JointConfig> &default_joints);

bool loadImuConfig(ros::NodeHandle &nh, std::array<double, 3> &imu_bias);

bool loadModelConfig(ros::NodeHandle &nh, ModelConfig &config);

void printModelConfig(const ModelConfig &config);

void processKeyboardInput(char ch, double lin_sensitivity, double ang_sensitivity);

void handleInput(double lin_sensitivity, double ang_sensitivity);

EulerAngle QuaternionToEuler(const Quaternion &q);

std::array<double, 3> quat_rotate_inverse(const std::array<double, 4> &quat, const std::array<double, 3> &vel);

std::vector<float> get_current_obs(std::vector<double> &init_pos, std::vector<float> &last_action);

std::vector<float> get_current_obs(std::vector<double> &init_pos, std::vector<float> &last_action, double cycle_count, double start_to_walk_cycle, bool with_history);

void callback(const std_msgs::Float64MultiArray::ConstPtr &msg);

#endif // HUMANOID_RL_INFERENCE_H