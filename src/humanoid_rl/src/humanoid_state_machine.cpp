#include "humanoid_state_machine.h"
#include "log4z.h"

HumanoidStateMachine::HumanoidStateMachine(ros::NodeHandle &nh) : nh_(nh), current_state_(DAMPING)
{
    state_sub_ = nh_.subscribe("/robot_state", 1, &HumanoidStateMachine::stateCallback, this);
    LOGI("HumanoidStateMachine object is being created, subscribed to /robot_state topic.");
}

void HumanoidStateMachine::stateCallback(const std_msgs::String::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::string state = msg->data;
    State new_state;

    if(state == "DAMPING")
    {
        new_state = DAMPING;
    } else if(state == "ZERO_POS")
    {
        new_state = ZERO_POS;
    } else if(state == "STAND")
    {
        new_state = STAND;
    } else if(state == "WALK")
    {
        new_state = WALK;
    } else
    {
        LOGFMTW("Unknown state: %s", state.c_str());
        return;
    }

    // 检查状态切换是否合法
    if(isValidTransition(current_state_, new_state))
    {
        current_state_ = new_state;
        LOGFMTI("State changed to: %s", state.c_str());
    } else
    {
        LOGFMTW("Invalid state transition from %s to %s", stateToString(current_state_).c_str(), state.c_str());
    }
}

HumanoidStateMachine::State HumanoidStateMachine::getCurrentState()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_state_;
}

bool HumanoidStateMachine::isValidTransition(State current, State next)
{
    switch(current)
    {
    case DAMPING:
        return next == ZERO_POS;
    case ZERO_POS:
        return next == DAMPING || next == STAND;
    case STAND:
        return next == ZERO_POS || next == WALK;
    case WALK:
        return next == STAND;
    default:
        return false;
    }
}

std::string HumanoidStateMachine::stateToString(State state)
{
    switch(state)
    {
    case DAMPING:
        return "DAMPING";
    case ZERO_POS:
        return "ZERO_POS";
    case STAND:
        return "STAND";
    case WALK:
        return "WALK";
    default:
        return "UNKNOWN";
    }
}