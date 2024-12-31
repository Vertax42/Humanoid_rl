#include <chrono>
#include <thread>
#include <fcntl.h>
#include <mutex>
#include <unistd.h>
#include <termios.h>
#include <ros/ros.h>
#include "humanoid_rl_inference.h"
#include "body_control.h"
#include "humanoid_policy.h"
#include "log4z.h"
#include "utils_logger.hpp"
#include "version.h"


using namespace zsummer::log4z;

std::mutex data_mutex; // data mutex for data synchronization
std::vector<double> measured_q_;
std::vector<double> measured_v_;
std::vector<double> measured_tau_;
std::array<double, 4> quat_est;
std::array<double, 3> angular_vel_local;
std::array<double, 3> imu_accel;

struct Command {
    // 1 for moving, 0 for standing used for yaw while stand
    double x, y, yaw, heading, move;
    Command(double _x = -0.2, double _y = 0.0, double _yaw = 0.0, double _heading = 0.64, double _move = 0.)
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

// 读取模型配置
bool loadModelConfig(ros::NodeHandle &nh, ModelConfig &config)
{
    bool success = true;
    success &= getParamHelper(nh, "model_path", config.model_path);
    success &= getParamHelper(nh, "obs_dim", config.obs_dim);
    success &= getParamHelper(nh, "action_dim", config.action_dim);
    success &= getParamHelper(nh, "history_length", config.history_length);
    success &= getParamHelper(nh, "obs_history_length", config.obs_history_length);
    success &= getParamHelper(nh, "lin_sensitivity", config.lin_sensitivity);
    success &= getParamHelper(nh, "ang_sensitivity", config.ang_sensitivity);
    return success;
}

// 打印模型配置
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

std::vector<float> get_current_obs()
{
    std::vector<float> tmp_obs; // 93
    // create tmp current data copy 7: neck_yaw_joint, 8: neck_pitch_joint, 17: waist_roll_joint
    std::vector<double> q; // 30
    std::vector<double> v; // 30
    std::vector<double> tau; // 30
    std::array<double, 4> quat;
    std::array<double, 3> angular_vel;
    std::array<double, 3> accel;
    {
        std::lock_guard<std::mutex> lock(data_mutex); // mutex lock
        q = measured_q_;
        v = measured_v_;
        tau = measured_tau_;
        quat = quat_est;
        angular_vel = angular_vel_local;
        accel = imu_accel;
    }

    // index of joints to remove
    std::vector<size_t> indices_to_remove = { 7, 8, 17 };

    // use std::remove_if and lambda to remove elements
    vec.erase(std::remove_if(vec.begin(), vec.end(),
                             [&indices_to_remove, index = 0](const auto &) mutable {
                                 return std::find(indices_to_remove.begin(), indices_to_remove.end(), index++)
                                        != indices_to_remove.end();
                             }),
              vec.end());
    // 1. linear velocity robot frame [lx, ly, lz], [3], 3 # Unoise(n_min=-0.1, n_max=0.1)
    // 2. angular velocity robot frame [ax, ay, az], [3], 6 # Unoise(n_min=-0.2, n_max=0.2)
    // 3. projected gravity robot frame [gx, gy, gz], [3], 9 # Unoise(n_min=-0.05, n_max=0.05)
    // 4. input commands [vx, vy, vyaw, vheading], [4], 13 # user_cmd.x, user_cmd.y, user_cmd.yaw, user_cmd.heading
    // 5. joint rel position [q1, q2, ..., q30], [27], 43 # default_joints_pos set to 0 Unoise(n_min=-0.01,
    // n_max=0.01)
    // 6. joint rel velocity [v1, v2, ..., v30], [27], 73 # default_joints_vel set to 0 Unoise(n_min=-1.5, n_max=1.5)
    // 7. last_action [a1, a2, ..., a27], [27], 94 # Unoise(n_min=-0.1, n_max=0.1)
    // now we do not use the headding observation so the obs_dim is 93
    
}

void callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{

    const int meanless_size = 6;                       // meanless size
    const int total_joints = meanless_size + N_JOINTS; // meanless + joint size
    const int quat_size = 4;                       // quat size
    const int angular_vel_size = 3;                // ang vel size
    const int imu_accel_size = 3;                  // IMU acc size
    
    std::lock_guard<std::mutex> lock(data_mutex); // lock mutex
    if(msg->data.size() < total_joints * 3 + quat_size + angular_vel_size + imu_accel_size)
    {
        LOGE("Invalid data size in Float64MultiArray message!");
        ROS_ERROR("Invalid data size in Float64MultiArray message!");
        return;
    }

    // position, velocity, torque
    measured_q_.assign(msg->data.begin(), msg->data.begin() + total_joints);

    measured_v_.assign(msg->data.begin() + total_joints, msg->data.begin() + 2 * total_joints);

    measured_tau_.assign(msg->data.begin() + 2 * total_joints, msg->data.begin() + 3 * total_joints);

    // decode quaternion
    std::copy(msg->data.begin() + 3 * total_joints, msg->data.begin() + 3 * total_joints + quat_size,
              quat_est.begin()); // 使用 quat_est.begin() 作为输出迭代器

    // decode angular velocity
    std::copy(msg->data.begin() + 3 * total_joints + quat_size,
              msg->data.begin() + 3 * total_joints + quat_size + angular_vel_size,
              angular_vel_local.begin()); // use angular_vel_local.begin() as output iterator

    // decode imu acceleration
    std::copy(msg->data.begin() + 3 * total_joints + quat_size + angular_vel_size,
              msg->data.begin() + 3 * total_joints + quat_size + angular_vel_size + imu_accel_size,
              imu_accel.begin()); // use imu_accel.begin() as output iterator
}

int main(int argc, char **argv)
{
    // log4z
    common_tools::clean_log_files(5);
    ILog4zManager::getRef().start();
    ILog4zManager::getRef().setLoggerLevel(LOG4Z_MAIN_LOGGER_ID, LOG_LEVEL_DEBUG);

    printf_program("Humanoid Policy Inference: for humanoid robot velocity rl policy inference on Xbot-L robot.");
    common_tools::printf_software_version("Humanoid Policy Inference");
    common_tools::dump_program_info_log4z("Humanoid Policy Inference");
    
    ros::init(argc, argv, "humanoid_policy_inference");
    ros::NodeHandle nh;

    std::vector<JointConfig> default_joints;
    XmlRpc::XmlRpcValue joint_list;
    if(nh.getParam("default_joints", joint_list))
    {
        if(joint_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
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
                default_joints.push_back(config);
            }
        }
    } else
    {
        LOGE("Failed to load joint configurations from parameter server!");
        return -1;
    }
    LOGFMTI("Loaded %lu joint configurations.", default_joints.size());

    ModelConfig config;
    if(loadModelConfig(nh, config))
    {
        printModelConfig(config);
    } else
    {
        LOGE("Failed to load model configuration from parameter server!");
        return -1;
    }

    // humanoid policy class
    // model_path, obs_dim, action_dim, history_length, obs_history_length
    HumanoidPolicy policy(config.model_path, config.obs_dim, config.action_dim, config.history_length, config.obs_history_length);

    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/policy_input", 1);
    ros::Subscriber sub = nh.subscribe("/controllers/xbot_controller/policy_output", 1, callback, ros::TransportHints().tcpNoDelay());

    std::thread inputThread(handleInput, config.lin_sensitivity, config.ang_sensitivity);
    while(ros::ok())
    {
        auto start_time = std::chrono::steady_clock::now();
        //...............................................todo How to get current_obs
        std::vector<float> current_obs(93, 0.0);
        std::vector<float> action = policy.inference(current_obs);
        for(long unsigned int i = 0; i < action.size(); i++)
        {
            LOGFMTD("Action[%lu]: %f", i, action[i]);
        }
        auto duration = std::chrono::steady_clock::now() - start_time;
        LOGFMTW("Inference time: %ld us", std::chrono::duration_cast<std::chrono::microseconds>(duration).count());
        auto micro_sec = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
        int sleep_time = 1000000 / CONTROL_FREQUENCY - micro_sec;
        LOGFMTW("Sleep time: %d us", sleep_time);
        std::this_thread::sleep_for(std::chrono::microseconds(sleep_time));

        ros::spinOnce();
    }
    
    LOGA("main quit........");
    return 0;
}