#ifndef HUMANOID_RL_INFERENCE_H
#define HUMANOID_RL_INFERENCE_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <array>
#include <std_msgs/Float64MultiArray.h>
#include "body_control.h"
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
    // 观测空间变量名列表
    std::vector<std::string> obs_names;
    // 动作关节角列表
    std::vector<std::string> action_joints;
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

bool is_valid_joint(std::vector<double> &measured_q, std::vector<JointConfig> &default_joint, float scale);

std::vector<float> get_current_obs(std::vector<double> &init_pos, std::vector<float> &last_action);

std::vector<float> get_current_obs(std::vector<double> &init_pos, std::vector<float> &last_action, double cycle_count, double start_to_walk_cycle, bool with_history);

void callback(const std_msgs::Float64MultiArray::ConstPtr &msg);

void callbackJoints(const sensor_msgs::JointState::ConstPtr &msg);

bool start_move = false;

std::vector<double> leftarm_joints(7, 0.0);

std::vector<double> rightarm_joints(7, 0.0);

std::vector<int> leftarmIndex = { 
    static_cast<int>(JointsNameWithHands::LEFT_ARM_YAW_JOINT),
    static_cast<int>(JointsNameWithHands::LEFT_ELBOW_PITCH_JOINT),
    static_cast<int>(JointsNameWithHands::LEFT_ELBOW_YAW_JOINT),
    static_cast<int>(JointsNameWithHands::LEFT_SHOULDER_PITCH_JOINT),
    static_cast<int>(JointsNameWithHands::LEFT_SHOULDER_ROLL_JOINT),
    static_cast<int>(JointsNameWithHands::LEFT_WRIST_ROLL_JOINT),
    static_cast<int>(JointsNameWithHands::LEFT_WRIST_YAW_JOINT)
};

std::vector<int> leftarmPubIndex= { 
    static_cast<int>(JointsName::LEFT_ARM_YAW_JOINT),       
    static_cast<int>(JointsName::LEFT_ELBOW_PITCH_JOINT),
    static_cast<int>(JointsName::LEFT_ELBOW_YAW_JOINT),     
    static_cast<int>(JointsName::LEFT_SHOULDER_PITCH_JOINT),
    static_cast<int>(JointsName::LEFT_SHOULDER_ROLL_JOINT), 
    static_cast<int>(JointsName::LEFT_WRIST_ROLL_JOINT),
    static_cast<int>(JointsName::LEFT_WRIST_YAW_JOINT)
};

std::vector<int> rightarmIndex = { 
    static_cast<int>(JointsNameWithHands::RIGHT_ARM_YAW_JOINT),
    static_cast<int>(JointsNameWithHands::RIGHT_ELBOW_PITCH_JOINT),
    static_cast<int>(JointsNameWithHands::RIGHT_ELBOW_YAW_JOINT),
    static_cast<int>(JointsNameWithHands::RIGHT_SHOULDER_PITCH_JOINT),
    static_cast<int>(JointsNameWithHands::RIGHT_SHOULDER_ROLL_JOINT),
    static_cast<int>(JointsNameWithHands::RIGHT_WRIST_ROLL_JOINT),
    static_cast<int>(JointsNameWithHands::RIGHT_WRIST_YAW_JOINT)
};

std::vector<int> rightarmPubIndex = {
    static_cast<int>(JointsName::RIGHT_ARM_YAW_JOINT),       
    static_cast<int>(JointsName::RIGHT_ELBOW_PITCH_JOINT),
    static_cast<int>(JointsName::RIGHT_ELBOW_YAW_JOINT),     
    static_cast<int>(JointsName::RIGHT_SHOULDER_PITCH_JOINT),
    static_cast<int>(JointsName::RIGHT_SHOULDER_ROLL_JOINT), 
    static_cast<int>(JointsName::RIGHT_WRIST_ROLL_JOINT),
    static_cast<int>(JointsName::RIGHT_WRIST_YAW_JOINT)
};

std::vector<double> arm_move_kp={50.0, 40.0, 25.0, 50.0, 15.0, 15.0, 15.0};

std::vector<double> arm_move_kd={25.0, 10.0, 15.0, 10.0, 10.0, 5.0, 10.0};

#endif // HUMANOID_RL_INFERENCE_H