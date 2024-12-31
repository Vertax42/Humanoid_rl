#ifndef HUMANOID_STATE_MACHINE_H
#define HUMANOID_STATE_MACHINE_H

#include <mutex>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>

class HumanoidStateMachine {
public:
    enum State { DAMPING, ZERO_POS, STAND, WALK };

    HumanoidStateMachine(ros::NodeHandle &nh);
    void stateCallback(const std_msgs::String::ConstPtr &msg);
    State getCurrentState();

private:
    bool isValidTransition(State current, State next);
    std::string stateToString(State state);

    ros::NodeHandle &nh_;
    ros::Subscriber state_sub_;
    State current_state_;
    std::mutex state_mutex_;
};

#endif // HUMANOID_STATE_MACHINE_H