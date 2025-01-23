#include "humanoid_rl_inference.h"
#include "body_control.h"
// #include "humanoid_policy.h"
#include "humanoid_state_machine.h"
#include "log4z.h"
#include "utils_logger.hpp"
#include "version.h"
#include <algorithm>
#include <chrono>
#include <fcntl.h>
#include <stdlib.h>
#include <mutex>
#include <ros/ros.h>
#include <termios.h>
#include <thread>
#include <unistd.h>


using namespace zsummer::log4z;

std::mutex data_mutex; // data mutex for data synchronization
std::mutex cmd_mutex; // command mutex for command synchronization

std::unordered_map<int, std::string> joint_table; // joint table [id, name]
// for callback function >>>
std::deque<double> hist_yaw_; // history yaw angle
std::vector<double> measured_q_; // measured joint position
std::vector<double> measured_v_; // measured joint velocity
std::vector<double> measured_tau_; // measured joint torque
std::array<double, 3> velocity_; // linear velocity
std::array<double, 4> quat_est_; // estimated quaternion
std::array<double, 3> angular_vel_; // angular velocity
std::array<double, 3> imu_accel_; // imu acceleration
std::array<double, 3> imu_bias_; // bias of the imu
std::array<double, 3> proj_grav_ = {0.0, 0.0, -0.981};// projected gravity z-axis
ros::Time last_time;
// <<< for callback function
std::vector<torch::jit::IValue> tensor;

struct Command {
    // 1 for moving, 0 for standing used for yaw while stand
    double x, y, yaw, heading, move;
    Command(double _x = 0.0, double _y = 0.0, double _yaw = 0.0, double _heading = 0.64, double _move = 1.0)
        : x(_x), y(_y), yaw(_yaw), heading(_heading), move(_move)
    {}
} user_cmd;

template <typename T>
T clip(T value, T min, T max)
{
    return std::max(min, std::min(value, max));
}

template <typename T>
bool getParamHelper(ros::NodeHandle &nh, const std::string &param_name, T &param_value)
{
    if(!nh.getParam(param_name, param_value))
    {
        LOGFMTE("Failed to load parameter: %s", param_name.c_str());
        return false;
    }
    return true;
}

bool loadJointConfigs(ros::NodeHandle &nh, const std::string &param_name, std::vector<JointConfig> &default_joints)
{
    XmlRpc::XmlRpcValue joint_list;
    if(!nh.getParam(param_name, joint_list))
    {
        LOGE("Failed to load joint configurations from parameter server!");
        return false;
    }

    if(joint_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        LOGE("Joint configurations parameter is not an array!");
        return false;
    }

    for(int i = 0; i < joint_list.size(); ++i)
    {
        XmlRpc::XmlRpcValue &joint = joint_list[i];
        JointConfig config;
        config.id = static_cast<int>(joint["index"]); // 读取 index
        config.name = static_cast<std::string>(joint["name"]);
        config.pos_des = static_cast<double>(joint["pos_des"]);
        config.vel_des = static_cast<double>(joint["vel_des"]);
        config.kp = static_cast<double>(joint["kp"]);
        config.kd = static_cast<double>(joint["kd"]);
        config.torque = static_cast<double>(joint["torque"]);
        config.upper_limit = static_cast<double>(joint["upper_limit"]);
        config.lower_limit = static_cast<double>(joint["lower_limit"]);
        // LOGFMTD("Joint index: %d, name: %s", config.id, config.name.c_str());
        default_joints.push_back(config);
    }

    LOGFMTI("Loaded %lu joint configurations successed!", default_joints.size());
    return true;
}

// load model configuration
bool loadModelConfig(ros::NodeHandle &nh, ModelConfig &config)
{
    bool success = true;
    success &= getParamHelper(nh, "model/model_path", config.model_path);
    success &= getParamHelper(nh, "model/obs_dim", config.obs_dim);
    success &= getParamHelper(nh, "model/action_dim", config.action_dim);
    success &= getParamHelper(nh, "model/history_length", config.history_length);
    success &= getParamHelper(nh, "model/obs_history_length", config.obs_history_length);
    success &= getParamHelper(nh, "model/lin_sensitivity", config.lin_sensitivity);
    success &= getParamHelper(nh, "model/ang_sensitivity", config.ang_sensitivity);
    return success;
}

// load imu configuration
bool loadImuConfig(ros::NodeHandle &nh, std::array<double, 3> &imu_bias)
{
    bool success = true;
    success &= getParamHelper(nh, "imu/bias_x", imu_bias[0]);
    success &= getParamHelper(nh, "imu/bias_y", imu_bias[1]);
    success &= getParamHelper(nh, "imu/bias_z", imu_bias[2]);
    return success;
}

// print model configuration
void printModelConfig(const ModelConfig &config)
{
    zsummer::log4z::ILog4zManager::getRef().setLoggerDisplay(LOG4Z_MAIN_LOGGER_ID, false);
    LOGD("=============================================================");
    LOGD("Loaded model configuration:");
    LOGFMTD("  model_path: %s", config.model_path.c_str());
    LOGFMTD("  obs_dim: %d", config.obs_dim);
    LOGFMTD("  action_dim: %d", config.action_dim);
    LOGFMTD("  history_length: %d", config.history_length);
    LOGFMTD("  obs_history_length: %d", config.obs_history_length);
    LOGD("=============================================================");
    std::cout << ANSI_COLOR_GREEN_BOLD << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cout << "Loaded model configuration:" << std::endl;
    std::cout << "    model_path: " << config.model_path << std::endl;
    std::cout << "    obs_dim: " << config.obs_dim << std::endl;
    std::cout << "    action_dim: " << config.action_dim << std::endl;
    std::cout << "    history_length: " << config.history_length << std::endl;
    std::cout << "    obs_history_length: " << config.obs_history_length << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << ANSI_COLOR_RESET << std::endl;
    zsummer::log4z::ILog4zManager::getRef().setLoggerDisplay(LOG4Z_MAIN_LOGGER_ID, true);
}

void processKeyboardInput(char ch, double lin_sensitivity, double ang_sensitivity)
{
    const double MAX_X = 0.6;
    const double MIN_X = -0.4;
    const double MAX_Y = 0.5;
    const double MIN_Y = -0.5;
    const double MAX_YAW = 0.5;
    const double MIN_YAW = -0.5;
    const double MAX_HEADING = 0.3;
    const double MIN_HEADING = -0.3;
    switch(ch)
    {
    case 'W': // increase +X linear velocity
        user_cmd.x = std::min(MAX_X, user_cmd.x + lin_sensitivity);
        break;
    case 'S': // decrease +X linear velocity
        user_cmd.x = std::max(MIN_X, user_cmd.x - lin_sensitivity);
        break;
    case 'A': // increase +Y linear velocity
        user_cmd.y = std::min(MAX_Y, user_cmd.y + lin_sensitivity);
        break;
    case 'D': // decrease +Y linear velocity
        user_cmd.y = std::max(MIN_Y, user_cmd.y - lin_sensitivity);
        break;
    case 'Q': // increase angular velocity (yaw rate)
        user_cmd.yaw = std::min(MAX_YAW, user_cmd.yaw + ang_sensitivity);
        break;
    case 'E': // decrease angular velocity (yaw rate)
        user_cmd.yaw = std::max(MIN_YAW, user_cmd.yaw - ang_sensitivity);
        break;
    case 'R': // reset all commands
        user_cmd.x = 0.;
        user_cmd.y = 0.;
        user_cmd.yaw = 0.;
        user_cmd.heading = 0.;
        user_cmd.move = 0.;
        break;
    case 'T': // increase heading
        user_cmd.yaw = std::min(MAX_HEADING, user_cmd.heading + ang_sensitivity);
        break;
    case 'G': // decrease heading
        user_cmd.yaw = std::max(MIN_HEADING, user_cmd.heading - ang_sensitivity);
        break;
    default:
        // undefined input
        break;
    }
    return;
}

void handleInput(double lin_sensitivity, double ang_sensitivity)
{
    // save the original terminal settings
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // set the terminal to unbuffered mode and turn off echo
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // set stdin to non-blocking mode
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    
    char ch;
    while(ros::ok())
    {
        if(read(STDIN_FILENO, &ch, 1) > 0)
        {
            processKeyboardInput(ch, lin_sensitivity, ang_sensitivity);
            if(ch == 'q') break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 关闭 ROS
    ros::shutdown();

    // 恢复原始终端设置
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return;
}

EulerAngle QuaternionToEuler(const Quaternion &q)
{
    EulerAngle angles;

    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if(std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

std::array<double, 3> quat_rotate_inverse(const std::array<double, 4> &quat, const std::array<double, 3> &vel)
{
    // quaternion: w, x, y, z: local to world
    // velocity: x, y, z: local frame

    constexpr double two = 2.0; // const

    const double w = quat[0];
    const double x = quat[1];
    const double y = quat[2];
    const double z = quat[3];

    const double vx = vel[0];
    const double vy = vel[1];
    const double vz = vel[2];

    // compute the tmp variables
    const double tmp1 = two * w * w - 1.0;
    const double tmp2 = two * w;
    const double tmp3 = two * (x * vx + y * vy + z * vz);

    // compute the projected velocity
    return { vx * tmp1 - (y * vz - z * vy) * tmp2 + x * tmp3, vy * tmp1 - (z * vx - x * vz) * tmp2 + y * tmp3,
             vz * tmp1 - (x * vy - y * vx) * tmp2 + z * tmp3 };
}

bool is_valid_joint(std::vector<double> &measured_q, std::vector<JointConfig> &default_joint, float scale)
{
    if(measured_q.size() != 36)
    {
        LOGE("Measured joint position size is not 36!");
        return false;
    }

    for(size_t i = 0; i < N_JOINTS; ++i)
    {
        if(measured_q[i + 6] < default_joint[i].lower_limit * scale || measured_q[i + 6] > default_joint[i].upper_limit * scale)
        {
            LOGFMTD("Joint %s is out of range: (%f, %f), current_measurement: %f", default_joint[i].name.c_str(), default_joint[i].lower_limit * scale, default_joint[i].upper_limit * scale, measured_q[i + 6]);
            return false;
        }
    }

    return true;
}

std::vector<float> get_current_obs(std::vector<double> &init_pos, std::vector<float> &last_action)
{
    zsummer::log4z::ILog4zManager::getRef().setLoggerDisplay(LOG4Z_MAIN_LOGGER_ID, false);
    std::vector<float> tmp_obs; // 93
    Command local_cmd;
    {
        std::lock_guard<std::mutex> cmd_lock(cmd_mutex);
        local_cmd = user_cmd; // Copy the shared command to a local variable
    }

    // create tmp current data copy 7: neck_yaw_joint, 8: neck_pitch_joint, 17: waist_roll_joint
    std::vector<double> q, tmp_q; // 30
    std::vector<double> v, tmp_v; // 30
    std::vector<double> tau, tmp_tau; // 30
    std::array<double, 4> quat; // 4
    std::array<double, 3> linear_vel; // 3 add the linear velocity
    std::array<double, 3> angular_vel; // 3
    std::array<double, 3> accel; // 3

    {
        std::lock_guard<std::mutex> lock(data_mutex); // mutex lock
        init_pos = measured_q_;
        q = measured_q_;
        v = measured_v_;
        tau = measured_tau_;
        quat = quat_est_;
        linear_vel = velocity_; // add the linear velocity
        angular_vel = angular_vel_;
        accel = imu_accel_;
    }

    if(q.empty() || v.empty() || tau.empty()) // check if the measured data is empty
    {   
        LOGFMTD("q, v, tau vector size: %ld, %ld, %ld", q.size(), v.size(), v.size());
        LOGE("Measured data (q, v, or tau) is empty.");
        // throw std::runtime_error("Measured data (q, v, or tau) is empty.");
        LOGE("Observation data not ready, return empty observation!");
        return std::vector<float>();
    }

    if(quat[0] == 0.0 && quat[1] == 0.0 && quat[2] == 0.0 && quat[3] == 0.0) // check if the quaternion is all zeros
    {
        throw std::runtime_error("Invalid quaternion (all zeros).");
        LOGE("Invalid quaternion (all zeros).");
        return std::vector<float>();
    }

    EulerAngle euler = QuaternionToEuler(Quaternion{ quat[0], quat[1], quat[2], quat[3] }); // convert quaternion to euler angle
    if(std::isnan(euler.yaw)) // check if the yaw angle is NaN
    {
        throw std::runtime_error("Invalid yaw angle (NaN).");
        LOGE("Invalid yaw angle (NaN).");
    }
    double current_yaw = euler.yaw * -1.0; // current yaw angle
    
    hist_yaw_.push_back(current_yaw); // push back the current yaw angle
    if(hist_yaw_.size() >= HIST_YAW_SIZE) 
        hist_yaw_.pop_front(); // pop the front element if the size is greater than 10
    double base_yaw = std::accumulate(hist_yaw_.begin(), hist_yaw_.end(), 0.0) / hist_yaw_.size(); // calculate the average yaw angle as the base yaw angle
    double target_yaw_angular
        = -0.5 * ((euler.yaw * -1.0 - base_yaw) - local_cmd.yaw) * local_cmd.move; // calculate the target yaw angular

    LOGFMTD("Before elements remove, q, v, tau have %ld, %ld, %ld elements!", q.size(), v.size(), tau.size());
    proj_grav_ = quat_rotate_inverse(quat, gravity);
    proj_grav_[0] *= -1.;

    // 1. linear velocity robot frame [lx, ly, lz], [3], 3 # Unoise(n_min=-0.1, n_max=0.1)
    // 2. angular velocity robot frame [ax, ay, az], [3], 6 # Unoise(n_min=-0.2, n_max=0.2)
    // 3. projected gravity robot frame [gx, gy, gz], [3], 9 # Unoise(n_min=-0.05, n_max=0.05)
    // 4. input commands [vx, vy, vyaw, ----vheading], [3], 12 # user_cmd.x, user_cmd.y, user_cmd.yaw, user_cmd.heading
    // 5. joint rel position [q1, q2, ..., q30], [27], 39 # default_joints_pos set to 0 Unoise(n_min=-0.01,
    // n_max=0.01)
    // 6. joint rel velocity [v1, v2, ..., v30], [27], 66 # default_joints_vel set to 0 Unoise(n_min=-1.5, n_max=1.5)
    // 7. last_action [a1, a2, ..., a27], [27], 93 # Unoise(n_min=-0.1, n_max=0.1)
    // now we do not use the headding observation so the obs_dim is 93

    for(int i = 0; i < 3; i++) // +3 = [3]
    {
        tmp_obs.push_back(linear_vel[i] * (i == 0 ? 1 : -1));
        LOGFMTD("Observation[%d]: base linear velocity[%d]: %f", i , i, linear_vel[i]);
    }

    for(int i = 0; i < 3; i++) // +3 = [6]
    {
        tmp_obs.push_back(angular_vel[i] * (i == 0 ? 1 : -1));
        LOGFMTD("Observation[%d]: base angular velocity[%d]: %f", i + 3, i, angular_vel[i]);
    }

    for(int i = 0; i < 3; i++) // +3 = [9]
    {
        tmp_obs.push_back(proj_grav_[i]);
        LOGFMTD("Observation[%d]: base projected gravity[%d]: %f", i + 6, i, proj_grav_[i]);
    }
    
    tmp_obs.push_back(local_cmd.x); // +1 = [10]
    LOGFMTD("Observation[%d]: user command linear_vel_x: %f", 9, local_cmd.x);
    tmp_obs.push_back(local_cmd.y); // +1 = [11]
    LOGFMTD("Observation[%d]: user command linear_vel_y: %f", 10, local_cmd.y);
    tmp_obs.push_back(target_yaw_angular); // +1 = [12]
    LOGFMTD("Observation[%d]: user command target_yaw_angular: %f", 11, target_yaw_angular);
    
    std::vector<size_t> skip_indices = {0, 1, 2, 3, 4, 5, 7+6, 8+6, 17+6};
    const size_t joint_index_map[] = {
        0, 1, 2, 3, 4, 5, 6,  // 0-6
        9, 10, 11, 12, 13, 14, 15, 16,  // skip 7,8
        18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29  // skip 17
    };
    
    for (size_t i = 0; i < q.size(); i++) {
        if (std::find(skip_indices.begin(), skip_indices.end(), i) != skip_indices.end()) {
            continue; // skip current element
        }
        tmp_obs.push_back(q[i]);
        LOGFMTD("Observation[%ld]: joint_%ld (%s) position: %f", i + 12, i - 6, joint_table[i - 6].c_str() ,q[i]);
    } // +27 = [39]
    for (size_t i = 0; i < v.size(); i++) {
        if (std::find(skip_indices.begin(), skip_indices.end(), i) != skip_indices.end()) {
            continue; // skip current element
        }
        tmp_obs.push_back(v[i]);
        LOGFMTD("Observation[%ld]: joint_%ld (%s) velocity: %f", i + 39, i - 6, joint_table[i - 6].c_str() ,v[i]);
    } // +27 = [66]
    for (size_t i = 0; i < last_action.size(); i++) {
        tmp_obs.push_back(last_action[i]);
        size_t joint_index = joint_index_map[i];
        LOGFMTD("Observation[%ld]: joint_%ld (%s) last_action: %f", i + 66, i, joint_table[joint_index].c_str(), last_action[i]);
    } // +27 = [93]

    LOGFMTD("tmp_observation's final size: %ld", tmp_obs.size());
    zsummer::log4z::ILog4zManager::getRef().setLoggerDisplay(LOG4Z_MAIN_LOGGER_ID, true);

    return tmp_obs;
}

std::vector<float> get_current_obs(std::vector<double> &init_pos, std::vector<float> &last_action, double cycle_count, double start_to_walk_cycle, bool with_history)
{
    zsummer::log4z::ILog4zManager::getRef().setLoggerDisplay(LOG4Z_MAIN_LOGGER_ID, false);
    std::vector<float> tmp_obs; // 47
    Command local_cmd;
    {
        std::lock_guard<std::mutex> cmd_lock(cmd_mutex);
        local_cmd = user_cmd; // Copy the shared command to a local variable
    }

    // create tmp current data copy 7: neck_yaw_joint, 8: neck_pitch_joint, 17: waist_roll_joint
    std::vector<double> q, tmp_q; // 30
    std::vector<double> v, tmp_v; // 30
    std::vector<double> tau, tmp_tau; // 30
    std::array<double, 4> quat; // 4
    std::array<double, 3> linear_vel; // 3 add the linear velocity
    std::array<double, 3> angular_vel; // 3
    std::array<double, 3> accel; // 3

    {
        std::lock_guard<std::mutex> lock(data_mutex); // mutex lock
        init_pos = measured_q_;
        q = measured_q_;
        v = measured_v_;
        tau = measured_tau_;
        quat = quat_est_;
        linear_vel = velocity_; // add the linear velocity
        angular_vel = angular_vel_;
        accel = imu_accel_;
    }

    if(q.empty() || v.empty() || tau.empty()) // check if the measured data is empty
    {   
        LOGFMTD("q, v, tau vector size: %ld, %ld, %ld", q.size(), v.size(), v.size());
        LOGE("Measured data (q, v, or tau) is empty.");
        // throw std::runtime_error("Measured data (q, v, or tau) is empty.");
        LOGE("Observation data not ready, return empty observation!");
        return std::vector<float>();
    }

    if(quat[0] == 0.0 && quat[1] == 0.0 && quat[2] == 0.0 && quat[3] == 0.0) // check if the quaternion is all zeros
    {
        throw std::runtime_error("Invalid quaternion (all zeros).");
        LOGE("Invalid quaternion (all zeros).");
        return std::vector<float>();
    }

    LOGFMTD("Before quaternion to euler, quat: %f, %f, %f, %f", quat[0], quat[1], quat[2], quat[3]);
    EulerAngle euler = QuaternionToEuler(Quaternion{ quat[0], quat[1], quat[2], quat[3] }); // convert quaternion to euler angle
    LOGFMTD("After quaternion to euler, euler: %f, %f, %f", euler.roll, euler.pitch, euler.yaw);
    if(std::isnan(euler.yaw)) // check if the yaw angle is NaN
    {
        throw std::runtime_error("Invalid yaw angle (NaN).");
        LOGE("Invalid yaw angle (NaN).");
    }
    double current_yaw = euler.yaw * -1.0; // current yaw angle

    LOGFMTD("Current yaw angle: %f", current_yaw);

    hist_yaw_.push_back(current_yaw); // push back the current yaw angle
    if(hist_yaw_.size() >= HIST_YAW_SIZE) 
        hist_yaw_.pop_front(); // pop the front element if the size is greater than 10
    double base_yaw = std::accumulate(hist_yaw_.begin(), hist_yaw_.end(), 0.0) / hist_yaw_.size(); // calculate the average yaw angle as the base yaw angle
    LOGFMTD("Base yaw angle: %f", base_yaw);
    double target_yaw_angular
        = -0.5 * ((euler.yaw * -1.0 - base_yaw) - local_cmd.yaw) * local_cmd.move; // calculate the target yaw angular
    target_yaw_angular = clip(target_yaw_angular, -0.4, 0.4);
    LOGFMTD("Target yaw angular: %f", target_yaw_angular);
    LOGFMTD("Before elements remove, q, v, tau have %ld, %ld, %ld elements!", q.size(), v.size(), tau.size());
    proj_grav_ = quat_rotate_inverse(quat, gravity);
    proj_grav_[0] *= -1.;

    double cycle_time = 0.64;
    double sin_p = std::sin(2 * PI * (cycle_count - start_to_walk_cycle) * (1.0 / CONTROL_FREQUENCY) / cycle_time);
    double cos_p = std::cos(2 * PI * (cycle_count - start_to_walk_cycle) * (1.0 / CONTROL_FREQUENCY) / cycle_time);

    LOGFMTD("Walking cycle progress: %f", (cycle_count - start_to_walk_cycle) * (1.0 / CONTROL_FREQUENCY) / cycle_time);
    LOGFMTD("Start walking cycle: %f, cycle count now: %f", start_to_walk_cycle, cycle_count);

    // 1. command_input  [sin, cos, vel_x, vel_y, aug_vel_yaw], 5
    // 2. leg joint position [q1, q2, ..., q12], 5 + 12 = 17
    // 3. leg joint velocity, [v1, v2, ..., v12], 17 + 12 = 29
    // 4. last actions, [a1, a2, ..., a12], 29 + 12 = 41
    // 5. base angular velocity, [wx, wy, wz], 41 + 3 = 44
    // 6. base euler angle, [roll, pitch, yaw], 44 + 3 = 47

    if(with_history)
    {
        tmp_obs.push_back(sin_p); // +1 = [1]
        tmp_obs.push_back(cos_p); // +1 = [2]
        LOGFMTD("Observation[%d]: sin_p: %f", 0, sin_p);
        LOGFMTD("Observation[%d]: cos_p: %f", 1, cos_p);
        tmp_obs.push_back(local_cmd.x); // +1 = [3]
        LOGFMTD("Observation[%d]: user command linear_vel_x: %f", 2, local_cmd.x);
        tmp_obs.push_back(local_cmd.y); // +1 = [4]
        LOGFMTD("Observation[%d]: user command linear_vel_y: %f", 3, local_cmd.y);
        tmp_obs.push_back(target_yaw_angular); // +1 = [5]
        LOGFMTD("Observation[%d]: user command cliped target_yaw_angular: %f", 4, target_yaw_angular);

        for(int i = 0; i < N_LEG_JOINTS; i++) {
            tmp_obs.push_back(1.0 * q[6 + N_HAND_JOINTS + i]); // full scale leg joint position
            LOGFMTD("Observation[%d]: joint_%d (%s) position: %f", i + 5 , N_HAND_JOINTS + i, joint_table[N_HAND_JOINTS + i].c_str(), q[6 + N_HAND_JOINTS + i]);
        } // +12 = [17]
        
        
        for(int i = 0; i < N_LEG_JOINTS; i++) {
            tmp_obs.push_back(0.05 * clip(v[6 + N_HAND_JOINTS + i], -14., 14.)); // 0.05 scale and clip the leg joint velocity
            LOGFMTD("Observation[%d]: joint_%d (%s) velocity: %f", i + 17 , N_HAND_JOINTS + i, joint_table[N_HAND_JOINTS + i].c_str(), v[6 + N_HAND_JOINTS + i]);
        } // +12 = [29]

        for(int i = 0; i < N_LEG_JOINTS; i++) {
            tmp_obs.push_back(last_action[i]);
            LOGFMTD("Observation[%d]: joint_%d (%s) last_action: %f", i + 29 , N_HAND_JOINTS + i, joint_table[N_HAND_JOINTS + i].c_str(), last_action[i]);
        } // +12 = [41]

        for(int i = 0; i < 3; i++) // +3 = [44]
        {
            tmp_obs.push_back(angular_vel[i] * (i > 0 ? 1 : -1)); // angular velocity from imu callback
            LOGFMTD("Observation[%d]: base angular velocity[%d]: %f", i + 41, i, angular_vel[i]);
        }
        
        tmp_obs.push_back(euler.roll); // +1 = [45]
        LOGFMTD("Observation[%d]: base euler angle roll: %f", 44, euler.roll);
        tmp_obs.push_back(-1.0 * euler.pitch); // +1 = [46]
        LOGFMTD("Observation[%d]: base euler angle pitch: %f", 45, -1.0 * euler.pitch);
        tmp_obs.push_back(-1.0 * euler.yaw - base_yaw); // +1 = [47]
        LOGFMTD("Observation[%d]: base euler angle yaw: %f", 46, (-1.0 * euler.yaw - base_yaw));
    }

    LOGFMTD("tmp_observation's final size: %ld", tmp_obs.size());
    zsummer::log4z::ILog4zManager::getRef().setLoggerDisplay(LOG4Z_MAIN_LOGGER_ID, true);

    return tmp_obs;
}

void callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    zsummer::log4z::ILog4zManager::getRef().setLoggerDisplay(LOG4Z_MAIN_LOGGER_ID, false);
    LOGFMTD("Callback function called!");
    const int meanless_size = 6;                       // meanless size
    const int total_joints = meanless_size + N_JOINTS; // meanless + joint size
    const int quat_size = 4;                       // quat size
    const int angular_vel_size = 3;                // ang vel size
    const int imu_accel_size = 3;                  // IMU acc size
    
    if(msg->data.size() < total_joints * 3 + quat_size + angular_vel_size + imu_accel_size)
    {
        LOGE("Invalid data size in Float64MultiArray message!");
        ROS_ERROR("Invalid data size in Float64MultiArray message!");
        return;
    }

    measured_q_.clear();
    measured_v_.clear();
    measured_tau_.clear();

    LOGFMTD("Received data size: %lu", msg->data.size());
    // position, velocity, torque
    measured_q_.assign(msg->data.begin(), msg->data.begin() + total_joints);
    LOGFMTD("Assigned measured_q_'s size: %ld", measured_q_.size());
    
    measured_v_.assign(msg->data.begin() + total_joints, msg->data.begin() + 2 * total_joints);
    LOGFMTD("Assigned measured_v_'s size: %ld", measured_v_.size());
    
    measured_tau_.assign(msg->data.begin() + 2 * total_joints, msg->data.begin() + 3 * total_joints);
    LOGFMTD("Assigned measured_tau_'s size: %ld", measured_q_.size());
    
    // decode quaternion
    std::copy(msg->data.begin() + 3 * total_joints, msg->data.begin() + 3 * total_joints + quat_size,
              quat_est_.begin()); // 使用 quat_est_.begin() 作为输出迭代器

    LOGFMTD("Received quaternion: (%f, %f, %f, %f)", quat_est_[0], quat_est_[1], quat_est_[2], quat_est_[3]);

    // decode angular velocity
    std::copy(msg->data.begin() + 3 * total_joints + quat_size,
              msg->data.begin() + 3 * total_joints + quat_size + angular_vel_size,
              angular_vel_.begin()); // use angular_vel_.begin() as output iterator

    LOGFMTD("Received angular velocity: (%f, %f, %f)", angular_vel_[0], angular_vel_[1], angular_vel_[2]);

    // decode imu linear acceleration
    std::copy(msg->data.begin() + 3 * total_joints + quat_size + angular_vel_size,
              msg->data.begin() + 3 * total_joints + quat_size + angular_vel_size + imu_accel_size,
              imu_accel_.begin()); // use imu_accel_.begin() as output iterator
    
    LOGFMTD("Received imu_accel: (%f, %f, %f)", imu_accel_[0], imu_accel_[1], imu_accel_[2]);

    // LOGE("Callback function!");
    if(measured_q_.empty() || measured_v_.empty() || measured_tau_.empty()) {
        LOGE("Callback function Error!!!!!!!!!!!!!");
        LOGE("+++++++++++++++++++++++++++++++++++++++");
    }

    // get current ros time
    ros::Time current_time = ros::Time::now();
    if (last_time.isZero()) { // check if the last time is zero
        last_time = current_time;
        // initialize the last_time
    }
    double dt = (current_time - last_time).toSec(); // calculate the time diff
    LOGFMTD("Time diff aka (current_time - last_time): %f", dt);
    last_time = current_time; // update the last time
    // LOGFMTD("Received imu_accel: (%f, %f, %f)", imu_accel_[0], imu_accel_[1], imu_accel_[2]);
    // gravity compensation for IMU acceleration
    std::array<double, 3> acceleration_corrected = {
        imu_accel_[0] + imu_bias_[0],
        imu_accel_[1] + imu_bias_[1], 
        imu_accel_[2] + imu_bias_[2]
    };

    // calculate linear velocity via IMU acceleration
    for (size_t i = 0; i < 3; ++i) {
        velocity_[i] += acceleration_corrected[i] * dt;
    }

    LOGFMTD("After integration, get velocity_[0]: %f, velocity_[1]: %f, velocity_[2]: %f", velocity_[0], velocity_[1], velocity_[2]);
    zsummer::log4z::ILog4zManager::getRef().setLoggerDisplay(LOG4Z_MAIN_LOGGER_ID, true);
    return;
}

void callbackJoints(const sensor_msgs::JointState::ConstPtr &msg)
{   
  
    leftarm_joints.clear();
    rightarm_joints.clear();
    int j = 0;
    int l = 0;
    for(size_t i = 0; i < msg->name.size(); ++i)
    {

        if(std::find(leftarmIndex.begin(), leftarmIndex.end(), i) != leftarmIndex.end())
        {
            leftarm_joints[j] = msg->position[i];
            j++;
        }
        // 判断 i 是否在 rightarmIndex 中
        else if(std::find(rightarmIndex.begin(), rightarmIndex.end(), i) != rightarmIndex.end())
        {
            rightarm_joints[l] = msg->position[i];
            l++;
        }
    }
    start_move = true;
}

int main(int argc, char **argv)
{
    // log4z  
    ILog4zManager::getRef().start();
    ILog4zManager::getRef().setLoggerLevel(LOG4Z_MAIN_LOGGER_ID, LOG_LEVEL_DEBUG);

    printf_program("Humanoid Policy Inference: for humanoid robot velocity rl policy inference on Xbot-L robot.");
    common_tools::printf_software_version("Humanoid Policy Inference");
    common_tools::dump_program_info_log4z("Humanoid Policy Inference");
    common_tools::clean_log_files(5);

    ros::init(argc, argv, "humanoid_policy_inference");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/policy_input", 1);
    ros::Subscriber sub = nh.subscribe("/controllers/xbot_controller/policy_output", 1, callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber joint_states_sub = nh.subscribe("/RXjoint_states", 1, callbackJoints, ros::TransportHints().tcpNoDelay());
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/xbot_joints_pub", 1);
    
    std::this_thread::sleep_for(std::chrono::seconds(3));
 
    std::vector<JointConfig> default_joints;
    if(!loadJointConfigs(nh, "default_joints", default_joints))
    {   
        return -1; // return -1 if failed to load joint configurations
    }

    // create a joint table of joint_id and joint_name
    for (const auto &joint : default_joints) {
        joint_table[joint.id] = joint.name;
    }
    
    // testing output
    // for (size_t i = 0; i < joint_table.size(); i++) {
    //     LOGFMTD("Joint_table ID: %ld, Joint_table Name: %s", i, joint_table[i].c_str());
    // }

    ModelConfig config;
    if(loadModelConfig(nh, config))
    {
        // decode $(env HOME)
        size_t env_pos = config.model_path.find("$(env HOME)");
        if(env_pos != std::string::npos)
        {
            const char *home_dir = getenv("HOME");
            config.model_path.replace(env_pos, 11, home_dir); // 11 is the length of "$(env HOME)"
        }
        printModelConfig(config);
    } else {
        LOGE("Failed to load model configuration from parameter server!");
        return -1;
    }

    if(loadImuConfig(nh, imu_bias_))
    {
        LOGFMTI("IMU bias: (%f, %f, %f)", imu_bias_[0], imu_bias_[1], imu_bias_[2]);
    } else {
        LOGE("Failed to load IMU bias from parameter server!");
        return -1;
    }
    // humanoid state machine class
    HumanoidStateMachine state_machine(nh);
    // humanoid policy class
    // model_path, obs_dim, action_dim, history_length, obs_history_length
    
    HumanoidPolicy policy(config.model_path, config.obs_dim, config.action_dim, config.history_length, config.obs_history_length);
    // bool warm_up = false;
    // while(!warm_up)
    // {   
    //     size_t current_obs_length = policy.get_CurrentObsLength();
    //     if(current_obs_length >= static_cast<size_t>(config.obs_history_length))
    //     {
    //         warm_up = true;
    //         LOGFMTI("Policy inference warm up finished, current observation history size: %ld", current_obs_length);
    //     } else {
    //         std::vector<float> obs(config.obs_dim, 0.0f);
    //         auto output = policy.inference(obs);
    //         LOGFMTD("Warm up policy inference, observation history size: %ld", current_obs_length);
    //     }
    // }

    std::thread inputThread(handleInput, config.lin_sensitivity, config.ang_sensitivity);
    std::vector<double> pos_des, vel_des, kp, kd, torque; // configure for the joint control parameters
    std::vector<float> current_obs; // current observation
    std::vector<float> last_action(config.action_dim, 0.0f);
    std::vector<float> action(config.action_dim, 0.0f);

    unsigned long cycle_count = 0;
    constexpr unsigned long startup_cycle = 100;
    constexpr unsigned long init_cycle = 500;
    constexpr unsigned long MAXCYCLE = 720000; // 2 hours
    
    bool initFlag = false;
    std_msgs::Float64MultiArray control_msg;

    // transition switch parameters
    bool is_transitioning = false;
    unsigned long transition_start_cycle = 0;
    constexpr unsigned long transition_duration = 500; // 5s（control frequency 100Hz）
    std::vector<double> init_pos;
    std::vector<double> stand_pos;
    ros::Rate rate(100); // control frequency 100Hz
    while(ros::ok())
    {   
        // static std::vector<double> default_pos;
        static unsigned long start_to_walk_cycle;
        // reset the joint control parameters
        pos_des.resize(N_JOINTS);
        vel_des.resize(N_JOINTS);
        kp.resize(N_JOINTS);
        kd.resize(N_JOINTS);
        torque.resize(N_JOINTS);

        current_obs.clear(); // reset the current observation
        action.clear(); // reset the action
        
        cycle_count++; // increase the cycle count
        LOGFMTD("Current cycle count: %ld", cycle_count);
        auto start_time = std::chrono::steady_clock::now();

        auto current_state = state_machine.getCurrentState(); // get the current state
        LOGFMTI("Current state machine is: %s", state_machine.stateToString(current_state).c_str());
        // wait until measured_data are not empty
        
        if((initFlag && (proj_grav_[2] > -0.8)) || (initFlag && !is_valid_joint(measured_q_, default_joints, 0.8)))// current_robot_state error
        {
            if(proj_grav_[2] > -0.8) {
                LOGFMTE("Project gravity z: %f", proj_grav_[2]);
                LOGFMTE("Robot state error: robot is not standing on the ground! Robot shutdown!");
            } else {
                LOGE("Robot state error: measured_q_ is invalid!");
                LOGE("Observed joint position reach limits! Robot shutdown!");
            }
            LOGE("Robot state error: send joint command as shown below:");
            
            for (int i = 0; i < N_JOINTS; i++)
            {
                pos_des[i] = 0.0;  // target zero position
                vel_des[i] = 0.0;  // target zero velocity
                kp[i] = 0.0;       // close position control
                kd[i] = 2.0;       // set damping value 2.0
                torque[i] = 0.0;   // zero torque
                // add logs for debug
                // LOGFMTD("Pub joint %d command message: pos_des = %.2f, vel_des = %.2f, kp = %.2f, kd = %.2f, torque = %.2f", i, pos_des[i], vel_des[i], kp[i], kd[i], torque[i]);
            }
            control_msg = BodyJointCommandWraper(pos_des, vel_des, kp, kd, torque, joint_state_pub);
            pub.publish(control_msg);
            LOGE("ROS Shut Down!");
            ros::shutdown();
            break;
        }

        if(cycle_count <= startup_cycle && !initFlag) {
            init_pos = measured_q_;
            LOGFMTD("In init loop: Before quaternion to euler, quat(w, x, y, z): %f, %f, %f, %f", quat_est_[0], quat_est_[1], quat_est_[2], quat_est_[3]);
            EulerAngle euler = QuaternionToEuler(Quaternion{ quat_est_[0], quat_est_[1], quat_est_[2], quat_est_[3] }); // convert quaternion to euler angle
            LOGFMTD("In init loop: After quaternion to euler, euler: %f, %f, %f", euler.roll, euler.pitch, euler.yaw);
            hist_yaw_.push_back(euler.yaw * -1.0);
            if(hist_yaw_.size() >= HIST_YAW_SIZE)
                hist_yaw_.pop_front();
            control_msg = BodyJointCommandWraper(pos_des, vel_des, kp, kd, torque, joint_state_pub);
            pub.publish(control_msg);
            LOGFMTI("In startup loop, time cycle: %ld", cycle_count);
            
            auto duration = std::chrono::steady_clock::now() - start_time;
            // auto micro_sec = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
            // int sleep_time = 1000000 / CONTROL_FREQUENCY - micro_sec;
            LOGFMTA("Startup loop cycle process time: %ld us", std::chrono::duration_cast<std::chrono::microseconds>(duration).count());
            // std::this_thread::sleep_for(std::chrono::microseconds(sleep_time));
            rate.sleep();
            ros::spinOnce();
            continue;
        } else if(cycle_count <= init_cycle && !initFlag) {
            
            // upper body init to damping mode
            double percentage = double(cycle_count - startup_cycle) / double(init_cycle - startup_cycle);
            double phase_scale = 0.0;

            if(percentage <= 0.5)
            {
                phase_scale = 1.0 - 2.0 * percentage; // 1.0 -> 0.0
            } else
            {
                phase_scale = 2.0 * (percentage - 0.5); // 0.0 -> 1.0
            }

            for(int i = 0; i < N_HAND_JOINTS; i++)
            {
                pos_des[i] = init_pos[6 + i] * phase_scale;
                vel_des[i] = 0.0;
                kp[i] = 200.0;
                kd[i] = 10.0;
                torque[i] = 0.0;
            }
            // lower body init to damping mode
            for(int i = 0; i < N_LEG_JOINTS; i++)
            {
                pos_des[N_HAND_JOINTS + i] = init_pos[6 + N_HAND_JOINTS + i] * phase_scale;
                vel_des[N_HAND_JOINTS + i] = 0.0;
                kp[N_HAND_JOINTS + i] = 300.0;
                kd[N_HAND_JOINTS + i] = 10.0;
                torque[N_HAND_JOINTS + i] = 0.0;
            }
            LOGFMTI("Transitioning from startup mode to init mode, percentage, phase_scale: %.6f, %.6f", percentage, phase_scale);
            control_msg = BodyJointCommandWraper(pos_des, vel_des, kp, kd, torque, joint_state_pub);
            pub.publish(control_msg);
            LOGFMTI("In init cycle loop, time cycle: %ld", cycle_count);

            auto duration = std::chrono::steady_clock::now() - start_time;
            // auto micro_sec = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
            // int sleep_time = 1000000 / CONTROL_FREQUENCY - micro_sec;
            LOGFMTA("Init loop cycle process time: %ld us", std::chrono::duration_cast<std::chrono::microseconds>(duration).count());
            // std::this_thread::sleep_for(std::chrono::microseconds(sleep_time));
            rate.sleep();
            ros::spinOnce();
            continue;
        } else if(cycle_count > init_cycle && !initFlag) {
            initFlag = true;
            LOGFMTI("Initialization finished in %ld cycles!", cycle_count - 1);
            LOGFMTI("Begin while loop at time cycle: %ld", cycle_count);
        }

        if(cycle_count >= MAXCYCLE)
        {
            LOGE("Reach the max cycle number, ROS shutdown!");
            ros::shutdown();
            break;
        }

        if(initFlag) {
            auto current_state = state_machine.getCurrentState(); // get the current state
            auto previous_state = state_machine.getPreviousState(); // get the previous state
            switch(current_state) {
                case HumanoidStateMachine::DAMPING:
                    // DAMPING STATE LOGIC
                    LOGI("DAMPING state logic...");
                    for(int i = 0; i < N_HAND_JOINTS; i++)
                    {
                        pos_des[i] = init_pos[6 + i];
                        vel_des[i] = 0.0;
                        kp[i] = 200.0;
                        kd[i] = 10.0;
                        torque[i] = 0.0;
                    }
                    // lower body init to damping mode
                    for(int i = 0; i < N_LEG_JOINTS; i++)
                    {
                        pos_des[N_HAND_JOINTS + i] = init_pos[6 + N_HAND_JOINTS + i];
                        vel_des[N_HAND_JOINTS + i] = 0.0;
                        kp[N_HAND_JOINTS + i] = 300.0;
                        kd[N_HAND_JOINTS + i] = 10.0;
                        torque[N_HAND_JOINTS + i] = 0.0;
                    }
                    control_msg = BodyJointCommandWraper(pos_des, vel_des, kp, kd, torque, joint_state_pub);
                    pub.publish(control_msg);
                    break;
                case HumanoidStateMachine::ZERO_POS:
                    // ZERO_POS STATE LOGIC
                    LOGI("ZERO_POS state logic...");
                    if(previous_state == HumanoidStateMachine::DAMPING && !is_transitioning)
                    {
                        // start transition
                        is_transitioning = true;
                        transition_start_cycle = cycle_count;
                        init_pos = measured_q_; // record the initial position of the transition
                    }
                    if(is_transitioning) {
                        double percentage = double(cycle_count - transition_start_cycle) / transition_duration;
                        if(percentage >= 1.0)
                        {
                            percentage = 1.0;
                            is_transitioning = false; // state transitioning down 

                            // after switch state，set previous_state to ZERO_POS
                            state_machine.setPreviousState(HumanoidStateMachine::ZERO_POS);
                            LOGI("Transition to ZERO_POS completed. Previous state updated from DAMPING to ZERO_POS.");
                        }
                        // interpolate the joint position
                        // upper body
                        for(int i = 0; i < N_HAND_JOINTS; i++) {
                            pos_des[i] = init_pos[6 + i] * (1 - percentage);
                            vel_des[i] = 0.0;
                            kp[i] = 200.0;
                            kd[i] = 10.0;
                            torque[i] = 0.0;
                        }
                        // lower body
                        for(int i = 0; i < N_LEG_JOINTS; i++) {
                            pos_des[N_HAND_JOINTS + i] = pos_des[N_HAND_JOINTS + i]
                                = init_pos[6 + N_HAND_JOINTS + i] * (1.0 - percentage);
                            vel_des[N_HAND_JOINTS + i] = 0.0;
                            kp[N_HAND_JOINTS + i] = 300.0;
                            kd[N_HAND_JOINTS + i] = 10.0;
                            torque[N_HAND_JOINTS + i] = 0.0;
                        }
                        control_msg = BodyJointCommandWraper(pos_des, vel_des, kp, kd, torque, joint_state_pub);
                        pub.publish(control_msg);
                        LOGFMTW("Transitioning from DAMPING mode to ZERO_POS mode, percentage: %.6f", percentage);
                        break;
                    } else {
                        for(int i = 0; i < N_HAND_JOINTS; i++)
                        {
                            pos_des[i] = 0.0;
                            vel_des[i] = 0.0;
                            kp[i] = 200.0;
                            kd[i] = 10.0;
                            torque[i] = 0.0;
                        }
                        // lower body init to damping mode
                        for(int i = 0; i < N_LEG_JOINTS; i++)
                        {
                            pos_des[N_HAND_JOINTS + i] = 0.0;
                            vel_des[N_HAND_JOINTS + i] = 0.0;
                            kp[N_HAND_JOINTS + i] = 300.0;
                            kd[N_HAND_JOINTS + i] = 10.0;
                            torque[N_HAND_JOINTS + i] = 0.0;
                        }
                        control_msg = BodyJointCommandWraper(pos_des, vel_des, kp, kd, torque, joint_state_pub);
                        pub.publish(control_msg);
                        LOGW("Normal ZERO_POS mode, keep sending zero position command.");
                        break;
                    }
                case HumanoidStateMachine::STAND:
                    // STAND STATE LOGIC
                    LOGI("STAND state logic...");
                    if(previous_state == HumanoidStateMachine::ZERO_POS)
                    {
                        stand_pos = measured_q_; // record the initial position of the transition
                        state_machine.setPreviousState(HumanoidStateMachine::STAND);
                    }
                    for(int i = 0; i < N_HAND_JOINTS; i++)
                    {
                        pos_des[i] = stand_pos[6 + i];
                        vel_des[i] = 0.0;
                        kp[i] = 200.0;
                        kd[i] = 10.0;
                        torque[i] = 0.0;
                    }
                    // lower body init to damping mode
                    for(int i = 0; i < N_LEG_JOINTS; i++)
                    {
                        pos_des[N_HAND_JOINTS + i] = stand_pos[6 + N_HAND_JOINTS + i];
                        vel_des[N_HAND_JOINTS + i] = 0.0;
                        kp[N_HAND_JOINTS + i] = 300.0;
                        kd[N_HAND_JOINTS + i] = 10.0;
                        torque[N_HAND_JOINTS + i] = 0.0;
                    }
                    control_msg = BodyJointCommandWraper(pos_des, vel_des, kp, kd, torque, joint_state_pub);
                    pub.publish(control_msg);
                    break;
                case HumanoidStateMachine::WALK:
                    // WALK STATE LOGIC
                    LOGI("WALK state logic...");
                    if(start_to_walk_cycle == 0)
                    {
                        start_to_walk_cycle = cycle_count;
                        LOGFMTI("Start to walk cycle: %ld", start_to_walk_cycle);
                    }
                    // How to get current_obs
                    // get the last action
                    last_action = policy.get_LastAction();
                    for(size_t i = 0; i < last_action.size(); i++)
                    {
                        LOGFMTD("Before get current obs, print last action[%ld]: %f", i, last_action[i]);
                    }
                    try {
                        if(config.action_dim == 27){
                            current_obs = get_current_obs(init_pos, last_action); // get the current observation [93] policy_test.pt
                        } else {
                            current_obs = get_current_obs(init_pos, last_action, cycle_count, start_to_walk_cycle, true); // get the current observation [47] policy_1.pt
                        }
                    } catch(const std::runtime_error &e) {
                        LOGE("ERROR in get_current_obs()!!"); 
                        // LOGFMTI("In main while loop, time cycle: %ld", cycle_count);
                        auto duration = std::chrono::steady_clock::now() - start_time;
                        // auto micro_sec = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
                        // int sleep_time = 1000000 / CONTROL_FREQUENCY - micro_sec;
                        LOGFMTA("Startup loop cycle process time: %ld us", std::chrono::duration_cast<std::chrono::microseconds>(duration).count());
                        // std::this_thread::sleep_for(std::chrono::microseconds(sleep_time));
                        rate.sleep();
                        ros::spinOnce();
                        continue;
                    }
                    
                    if(config.action_dim == 27) {
                        action = policy.inference(current_obs); // action is a vector of float 27 dim
                        policy.update_History(current_obs, action); // update the history
                    
                        for(size_t i = 0; i < action.size(); i++)
                        {
                            action[i] = std::min(MAX_CLIP, std::max(MIN_CLIP, action[i]));
                        }
                        // std::vector<size_t> upperbody_skip_indices = { 7, 8, 17 };
                        action.insert(action.begin() + 17, 0); // 在位置17插入0 WAIST_ROLL_JOINT
                        action.insert(action.begin() + 8, 0);  // 在位置8插入0 NECK_PITCH_JOINT
                        action.insert(action.begin() + 7, 0);  // 在位置7插入0 NECK_YAW_JOINT

                        for(int i = 0; i < N_HAND_JOINTS; i++)
                        {
                            kp[i] = 300.0;
                            kd[i] = 20.0;
                            torque[i] = 0.0;
                            pos_des[i] = action[i] * 0.3;
                            vel_des[i] = 0.0;

                            if(i == 16 || i == 17)
                            {
                                kp[i] = 200.0; 
                                kd[i] = 40.0;
                            }
                        }
                        // lower body init to damping mode
                        for(int i = 0; i < N_LEG_JOINTS; i++)
                        {
                            kp[N_HAND_JOINTS + i] = 200.0;
                            kd[N_HAND_JOINTS + i] = 10.0;
                            torque[N_HAND_JOINTS + i] = 0.0;
                            pos_des[i] = action[i] * 0.3;
                            vel_des[N_HAND_JOINTS + i] = 0.0;
                        }
                        // left_leg_pitch_joint && right_leg_pitch_joint
                        for(int i :{20 ,26})
                        {
                            kp[i] = 350.0;
                        }
                        // left_knee_joint && right_knee_joint
                        for(int i :{21, 27})
                        {
                            kp[i] = 350.0;
                        }
                        // foot ankle joints
                        for(int i :{22, 23, 28, 29})
                        {
                            kp[i] = 15.0;
                        }
                        control_msg = BodyJointCommandWraper(pos_des, vel_des, kp, kd, torque, joint_state_pub);
                        for(size_t i = 0; i < action.size(); i++)
                        {
                            LOGFMTD("Action[%lu] output by * 0.3: %f", i, action[i] * 0.3);
                        }
                    } else {
                        // action dim 12
                        action = policy.inference(current_obs, true); // action is a vector of float 12 dim
                        // policy.set_LastAction(action); // update the last action
                        for(size_t i = 0; i < action.size(); i++)
                        {
                            action[i] = std::min(MAX_CLIP, std::max(MIN_CLIP, action[i]));
                        }
                        policy.set_LastAction(action); // update the last action
                        // upper body joint control
                        for (int i = 0; i < N_HAND_JOINTS; i++)
                        {
                            kp[i] = 200.0;
                            kd[i] = 10.0;
                            pos_des[i] = 0.0;
                            vel_des[i] = 0.0;
                            torque[i] = 0.0;
                            // waist joint control
                            if (i == 16 || i == 17)
                            {
                            kp[i] = 400.0;
                            kd[i] = 10.0;
                            }
                        }
                        // leg joint control
                        for (int i = 0; i < N_LEG_JOINTS; i++)
                        {
                            kp[N_HAND_JOINTS + i] = 200.0;
                            kd[N_HAND_JOINTS + i] = 10.0;
                            pos_des[N_HAND_JOINTS + i] = 0.25 * action[i];
                            vel_des[N_HAND_JOINTS + i] = 0.0;
                            torque[N_HAND_JOINTS + i] = 0.0;
                        }
                        // elbow joint control
                        for (int i : {4, 5, 10, 11})
                        {
                            kp[N_HAND_JOINTS + i] = 15.0;
                            kd[N_HAND_JOINTS + i] = 10.0;
                        }
                        // shoulder joint control
                        for (int i : {2, 3, 8, 9})
                        {
                            kp[N_HAND_JOINTS + i] = 350.0;
                            kd[N_HAND_JOINTS + i] = 10.0;
                        }
                        control_msg = BodyJointCommandWraper(pos_des, vel_des, kp, kd, torque, joint_state_pub);
                        for(size_t i = 0; i < action.size(); i++)
                        {
                            LOGFMTD("Action[%lu] output BY * 0.25: %f", i, action[i] * 0.25);
                        }
                    }
                    // pub.publish(control_msg);
                    LOGW("WALK mode, keep sending policy output command.");
                    break;
                default:
                    LOGE("Unknown state!");
                    ROS_WARN("Unknown state!");
                    break;
            }
        }
        
        auto duration = std::chrono::steady_clock::now() - start_time;
        auto micro_sec = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
        if(current_state == HumanoidStateMachine::WALK) {
            LOGFMTA("The whole inference time: %ld us", micro_sec);
        }
        else {
            LOGFMTA("The whole cycle process time: %ld us", micro_sec);
        }
        // int sleep_time = 1000000 / CONTROL_FREQUENCY - micro_sec;
        // LOGFMTW("Cycle sleep time: %d us", sleep_time);
        // std::this_thread::sleep_for(std::chrono::microseconds(sleep_time));
        rate.sleep();
        ros::spinOnce();
    }
    if(inputThread.joinable())
        inputThread.join();
    
    LOGA("main quit........");
    return 0;
}