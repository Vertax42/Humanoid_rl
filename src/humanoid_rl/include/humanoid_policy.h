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
    // 构造函数
    HumanoidPolicy(const std::string& model_path, int obs_dim, int action_dim, int history_length, int obs_history_length = 1);

    // 析构函数
    ~HumanoidPolicy();

    // 推理函数，输入当前观测，返回动作
    std::vector<float> inference(const std::vector<float>& observation);

    // 回调预热 && 保存历史观测
    size_t record_ObsHistory(const std::vector<float>& observation);

    // 获取历史观测和动作
    const std::deque<std::vector<float>>& get_ObsHistory() const;
    const std::vector<float>& get_LastAction() const;
    size_t get_CurrentObsLength() const;
    
    // 打印模型结构
    size_t count_parameters(const torch::jit::script::Module &model) const;
    void print_model_summary(const torch::jit::script::Module &model) const;
    void print_Model() const;

private:
    // 模型
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