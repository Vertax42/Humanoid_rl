#!/usr/bin/env python

import rosbag
import rospy
from std_msgs.msg import Float64MultiArray

# 定义关节名称
joint_names = [
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
    "right_ankle_roll_joint",
]


def extract_joint_positions(bag_file):
    # 打开 rosbag 文件
    bag = rosbag.Bag(bag_file)

    # 遍历 /policy_input 话题的消息
    for topic, msg, t in bag.read_messages(topics=["/policy_input"]):
        # 提取前 12 个数字
        des_pos = msg.data[:12]

        # 打印关节名称和对应的 des_pos
        for i, joint_name in enumerate(joint_names):
            print(f"{joint_name}: {des_pos[i]}")

        # 打印分隔线
        print("-" * 40)

    # 关闭 rosbag 文件
    bag.close()


if __name__ == "__main__":
    # 设置 rosbag 文件路径
    bag_file = "action_output.bag"

    # 提取并打印关节位置
    extract_joint_positions(bag_file)
