#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
import tf
import tf.transformations
import numpy as np
import scipy.spatial.transform as spt

ins_hand_upper_from_thumb = [2.878, 0.9878, 3.0718, 3.0718, 3.0718, 3.0718, 2.878, 0.9878, 3.0718, 3.0718, 3.0718, 3.0718]  # 上限，从大拇指开始，左手在前
ins_hand_lower_from_thumb = [1.58825, -0.2094, 0.349, 0.349, 0.349, 0.349, 1.58825, -0.2094, 0.349, 0.349, 0.349, 0.349]  # 下限，从大拇指开始，左手在前

def handsNormalize(hand_pos, hand_pos_norm):
    if len(hand_pos) != 12 or len(hand_pos_norm) != 12:
        print(f"hand_pos.size(): {len(hand_pos)}")
        print(f"hand_pos_norm.size(): {len(hand_pos_norm)}")
        print("err: [handsNormalize] hand_pos.size() != 12) || (hand_pos_norm.size() != 12)")
        return

    for j in range(12):
        hand_pos_norm[j] = (hand_pos[j] - ins_hand_lower_from_thumb[j]) / (ins_hand_upper_from_thumb[j] - ins_hand_lower_from_thumb[j])

    return

def handsDenormalize(hand_pos_norm, hand_pos):
    if len(hand_pos_norm) != 12 or len(hand_pos) != 12:
        print(f"hand_pos_norm.size(): {len(hand_pos_norm)}")
        print(f"hand_pos.size(): {len(hand_pos)}")
        print("err: [handsDenormalize] hand_pos_norm.size() != 12) || (hand_pos.size() != 12)")
        return

    for i in range(12):
        hand_pos[i] = hand_pos_norm[i] * (ins_hand_upper_from_thumb[i] - ins_hand_lower_from_thumb[i]) + ins_hand_lower_from_thumb[i]

    return
def main():
    rospy.init_node('test_hand_node')    
    hand_cmd_pub = rospy.Publisher('ins_hand_cmd', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(1) # 
    test_id = 0 #id for hand to test test_id [0~11]

    cnt = 0
    hand_pos_norm = np.zeros(12)  
    for i in range(12):
        hand_pos_norm[i]= 1.0
    while not rospy.is_shutdown():  
        if test_id <0 or test_id > 11:
            print("test_id should be in [0,11]")
            test_id = 0


        if cnt % 2 == 0:
            if(hand_pos_norm[test_id] == 0):
                hand_pos_norm[test_id] = 1
            elif(hand_pos_norm[test_id] == 1):
                hand_pos_norm[test_id] = 0
        # hand_pos_norm[test_id] = 1.0
        hand_cmd = Float64MultiArray()
        
        hand_cmd.data = np.zeros(12)  
        handsDenormalize(hand_pos_norm, hand_cmd.data)
        print('hand_cmd.data',hand_cmd.data)    

        
        hand_cmd_pub.publish(hand_cmd)   
        cnt = cnt + 1
        if cnt >4:
            hand_pos_norm[test_id] = 1
            test_id = test_id + 1
            cnt = 0
        print('test_id',test_id)
        rate.sleep()

if __name__ == '__main__':
    main()
