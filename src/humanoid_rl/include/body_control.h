#ifndef BODY_CONTROL_H
#define BODY_CONTROL_H
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <vector>

constexpr double PI = 3.14159265358979323846;
constexpr int CONTROL_FREQUENCY = 100;
constexpr int N_JOINTS = 30;
constexpr int N_HAND_JOINTS = 18; // left arm(7) + neck(2) + right arm(7) + waist(2)
constexpr int N_LEG_JOINTS = 12;  // left leg(6) + right leg(6)
constexpr std::array<double, 3> gravity = { 0.0, 0.0, -0.981 }; // gravity vector
constexpr float MAX_CLIP = 18.0;
constexpr float MIN_CLIP = -18.0;
constexpr size_t HIST_YAW_SIZE = 50;

std_msgs::Float64MultiArray BodyJointCommandWraper(std::vector<double> &pos_des, std::vector<double> &vel_des, std::vector<double> &kp, std::vector<double> &kd, std::vector<double> &torque, ros::Publisher &joint_state_pub);

static const std::vector<std::string> rviz_joints_name = { 
  "neck_yaw_joint",
  "neck_pitch_joint",
  "left_shoulder_pitch_joint",
  "left_shoulder_roll_joint",
  "left_arm_yaw_joint",
  "left_elbow_pitch_joint",
  "left_elbow_yaw_joint",
  "left_wrist_roll_joint",
  "left_wrist_yaw_joint",
  "right_shoulder_pitch_joint",
  "right_shoulder_roll_joint",
  "right_arm_yaw_joint",
  "right_elbow_pitch_joint",
  "right_elbow_yaw_joint",
  "right_wrist_roll_joint",
  "right_wrist_yaw_joint",
  "waist_yaw_joint",
  "waist_roll_joint",
  "left_leg_roll_joint",
  "left_leg_yaw_joint",
  "left_leg_pitch_joint",
  "left_knee_joint",
  "left_ankle_pitch_joint",
  "left_ankle_roll_joint",
  "right_leg_roll_joint",
  "right_leg_yaw_joint",
  "right_leg_pitch_joint",
  "right_knee_joint",
  "right_ankle_pitch_joint",
  "right_ankle_roll_joint"
};

enum class JointsName {
    LEFT_SHOULDER_PITCH_JOINT,
    LEFT_SHOULDER_ROLL_JOINT,
    LEFT_ARM_YAW_JOINT,
    LEFT_ELBOW_PITCH_JOINT,
    LEFT_ELBOW_YAW_JOINT,
    LEFT_WRIST_ROLL_JOINT,
    LEFT_WRIST_YAW_JOINT,
    NECK_YAW_JOINT,
    NECK_PITCH_JOINT,
    RIGHT_SHOULDER_PITCH_JOINT,
    RIGHT_SHOULDER_ROLL_JOINT,
    RIGHT_ARM_YAW_JOINT,
    RIGHT_ELBOW_PITCH_JOINT,
    RIGHT_ELBOW_YAW_JOINT,
    RIGHT_WRIST_ROLL_JOINT,
    RIGHT_WRIST_YAW_JOINT,
    WAIST_YAW_JOINT,
    WAIST_ROLL_JOINT,
    LEFT_LEG_ROLL_JOINT,
    LEFT_LEG_YAW_JOINT,
    LEFT_LEG_PITCH_JOINT,
    LEFT_KNEE_JOINT,
    LEFT_ANKLE_PITCH_JOINT,
    LEFT_ANKLE_ROLL_JOINT,
    RIGHT_LEG_ROLL_JOINT,
    RIGHT_LEG_YAW_JOINT,
    RIGHT_LEG_PITCH_JOINT,
    RIGHT_KNEE_JOINT,
    RIGHT_ANKLE_PITCH_JOINT,
    RIGHT_ANKLE_ROLL_JOINT
};

enum class JointsNameWithHands {
    LEFT_ANKLE_PITCH_MOTOR1_JOINT,
    LEFT_ANKLE_PITCH_MOTOR2_JOINT,
    LEFT_ARM_YAW_JOINT,
    LEFT_ELBOW_PITCH_JOINT,
    LEFT_ELBOW_YAW_JOINT,
    LEFT_HAND_INDEX_BEND_JOINT,
    LEFT_HAND_INDEX_JOINT1,
    LEFT_HAND_INDEX_JOINT2,
    LEFT_HAND_MID_JOINT1,
    LEFT_HAND_MID_JOINT2,
    LEFT_HAND_PINKY_JOINT1,
    LEFT_HAND_PINKY_JOINT2,
    LEFT_HAND_RING_JOINT1,
    LEFT_HAND_RING_JOINT2,
    LEFT_HAND_THUMB_BEND_JOINT,
    LEFT_HAND_THUMB_ROTA_JOINT1,
    LEFT_HAND_THUMB_ROTA_JOINT2,
    LEFT_KNEE_MOTOR_JOINT,
    LEFT_LEG_PITCH_JOINT,
    LEFT_LEG_ROLL_JOINT,
    LEFT_LEG_YAW_JOINT,
    LEFT_SHOULDER_PITCH_JOINT,
    LEFT_SHOULDER_ROLL_JOINT,
    LEFT_WRIST_ROLL_JOINT,
    LEFT_WRIST_YAW_JOINT,
    NECK_PITCH_JOINT,
    NECK_YAW_JOINT,
    RIGHT_ANKLE_PITCH_MOTOR1_JOINT,
    RIGHT_ANKLE_PITCH_MOTOR2_JOINT,
    RIGHT_ARM_YAW_JOINT,
    RIGHT_ELBOW_PITCH_JOINT,
    RIGHT_ELBOW_YAW_JOINT,
    RIGHT_HAND_INDEX_BEND_JOINT,
    RIGHT_HAND_INDEX_JOINT1,
    RIGHT_HAND_INDEX_JOINT2,
    RIGHT_HAND_MID_JOINT1,
    RIGHT_HAND_MID_JOINT2,
    RIGHT_HAND_PINKY_JOINT1,
    RIGHT_HAND_PINKY_JOINT2,
    RIGHT_HAND_RING_JOINT1,
    RIGHT_HAND_RING_JOINT2,
    RIGHT_HAND_THUMB_BEND_JOINT,
    RIGHT_HAND_THUMB_ROTA_JOINT1,
    RIGHT_HAND_THUMB_ROTA_JOINT2,
    RIGHT_KNEE_MOTOR_JOINT,
    RIGHT_LEG_PITCH_JOINT,
    RIGHT_LEG_ROLL_JOINT,
    RIGHT_LEG_YAW_JOINT,
    RIGHT_SHOULDER_PITCH_JOINT,
    RIGHT_SHOULDER_ROLL_JOINT,
    RIGHT_WRIST_ROLL_JOINT,
    RIGHT_WRIST_YAW_JOINT,
    WAIST_ROLL_JOINT,
    WAIST_YAW_JOINT
};


#endif // BODY_CONTROL_H