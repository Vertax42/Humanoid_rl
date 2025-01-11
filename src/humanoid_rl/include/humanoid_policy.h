#ifndef HUMANOID_POLICY_H
#define HUMANOID_POLICY_H

#include <deque>
#include <vector>
#include <string>
#include <mutex>
#include <torch/torch.h>
#include <torch/script.h>
#include <ros/ros.h>
#include "utils_common.h"
class HumanoidPolicy {
public:
    // constructor
    HumanoidPolicy(const std::string& model_path, int obs_dim, int action_dim, int history_length, int obs_history_length = 1);

    // destructor
    ~HumanoidPolicy();

    // inference *action* from *observation* with_history flag
    std::vector<float> inference(const std::vector<float>& observation, bool with_history = false);

    // record history observation
    size_t record_ObsHistory(const std::vector<float>& observation);

    // get history observation and last action
    const std::deque<std::vector<float>>& get_ObsHistory() const;
    const std::vector<float>& get_LastAction() const;
    size_t get_CurrentObsLength() const;
    
    // print model summary
    size_t count_parameters(const torch::jit::script::Module &model) const;
    void print_model_summary(const torch::jit::script::Module &model) const;
    void print_Model() const;

private:
    // model
    torch::jit::script::Module _model;

    // input and output dimensions
    size_t _obs_dim;
    size_t _action_dim;

    // record_length
    size_t _history_length;
    size_t _obs_history_length;

    // history of observations and actions
    std::deque<std::vector<float>> _obs_history;
    std::vector<float> _last_action;

    // update history
    void update_History(const std::vector<float>& obs, const std::vector<float>& action);
};

#endif // HUMANOID_POLICY_H