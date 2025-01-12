#include <sstream>
#include "humanoid_policy.h"
#include "log4z.h"

HumanoidPolicy::HumanoidPolicy(const std::string& model_path, int obs_dim, int action_dim, int history_length, int obs_history_length)
    : _obs_dim(obs_dim), _action_dim(action_dim), _history_length(history_length), _obs_history_length(obs_history_length) {
    try {
        // load_model
        _model = torch::jit::load(model_path);
        _model.eval();
    } catch (const c10::Error& e) {
        throw std::runtime_error("Failed to load model: " + model_path);
    }
    if (_obs_dim <= 0 || _action_dim <= 0 || _history_length <= 0) {
        throw std::invalid_argument("Invalid dimensions or history length provided.");
    }
    // initialize history && last_action
    _obs_history.resize(_obs_history_length, std::vector<float>(_obs_dim, 0.0f));
    _last_action.resize(_action_dim, 0.0f);
    // logInfo && print parameters
    LOGI("=============================================================");
    LOGFMTI("HumanoidPolicy object is being created.");
    LOGFMTI("Model path: %s", model_path.c_str());
    LOGFMTI("Observation dimension: %ld", _obs_dim);
    LOGFMTI("Action dimension: %ld", _action_dim);
    LOGFMTI("History length: %ld", _history_length);
    LOGFMTI("Observation history length: %ld", _obs_history_length);
    LOGI("=============================================================");
    print_Model();
}

HumanoidPolicy::~HumanoidPolicy()
{
    // Do nothing
    std::cout << "HumanoidPolicy object is being destroyed." << std::endl;
}

// inference
std::vector<float> HumanoidPolicy::inference(const std::vector<float>& observation, bool with_history) {
    // check observation size
    if (observation.size() != _obs_dim) {
        throw std::invalid_argument("Invalid observation size.");
    }

    if(with_history) {
        torch::Tensor input_tensor = torch::zeros({_obs_dim * _obs_history_length}, torch::kFloat32); // [47 x 15] [705]
        _obs_history.push_back(observation); // only update observation history
        _obs_history.pop_front(); // pop the front element
        // flatten observation history
        for (size_t i = 0; i < _obs_history.size(); i++) {
            for (size_t j = 0; j < _obs_dim; j++) {
                input_tensor.index_put_({i * (int)_obs_history[0].size() + j}, _obs_history[i][j]);
            }
        }
        LOGFMTD("After flatten observation history, input_tensor size: %ld", input_tensor.size(0));
    } else {
        // convert observation to tensor, add batch dimension
        torch::Tensor input_tensor = torch::tensor(observation, torch::kFloat32).unsqueeze(0).contiguous(); // [1, obs_dim] [1, 93]
    }
    
    // observation tensor preprocessing
    
    // get action tensor
    torch::Tensor action_tensor = _model.forward({input_tensor}).toTensor(); // [1, action_dim] [1, 27]
    // convert action tensor to vector
    std::vector<float> action(_action_dim);
    std::memcpy(action.data(), action_tensor.data_ptr(), _action_dim * sizeof(float));
    // update history
    if(with_history) {
        _last_action = action; // because the observation history has been updated
    } else {
        update_History(observation, action);
    }
    // update_History(observation, action);
    return action;
}

// record history observation
size_t HumanoidPolicy::record_ObsHistory(const std::vector<float>& observation){
    _obs_history.push_back(observation);
    if (_obs_history.size() >= _history_length) {
        _obs_history.pop_front();
    }
    return _obs_history.size();
}

// get history observation
const std::deque<std::vector<float>>& HumanoidPolicy::get_ObsHistory() const {
    return _obs_history;
}

// get last action
const std::vector<float>& HumanoidPolicy::get_LastAction() const {
    return _last_action;
}

// get current observation length
size_t HumanoidPolicy::get_CurrentObsLength() const {
    return _obs_history.size();
}

size_t HumanoidPolicy::count_parameters(const torch::jit::script::Module &model) const
{
    size_t num_params = 0;
    for(const auto &pair : model.named_parameters())
    {
        const torch::Tensor &param = pair.value;
        num_params += param.numel(); // numel() 返回张量的元素数量
    }
    return num_params;
}

void HumanoidPolicy::print_model_summary(const torch::jit::script::Module &model) const
{
    size_t num_layers = 0;
    std::cout << ANSI_COLOR_PURPLE_BOLD << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    LOGD("=============================================================");
    for(const auto &pair : model.named_parameters())
    {
        const std::string &name = pair.name;
        const torch::Tensor &param = pair.value;
        std::ostringstream oss;
        oss << "[";
        for(size_t i = 0; i < param.sizes().size(); ++i)
        {
            oss << param.sizes()[i];
            if(i < param.sizes().size() - 1)
            {
                oss << ", ";
            }
        }
        oss << "]";
         // std::cout << "Layer: " << name << ", Shape: " << param.sizes() << std::endl;
        LOGFMTD("Layer: %s, Shape: %s", name.c_str(), oss.str().c_str());
        std::cout << "    Layer: " << name << ", Shape: " << oss.str() << std::endl;
        num_layers++;
    }
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << ANSI_COLOR_RESET
              << std::endl;
    LOGD("=============================================================");
    LOGFMTI("Total layers: %lu", num_layers);
}

void HumanoidPolicy::print_Model() const
{
    zsummer::log4z::ILog4zManager::getRef().setLoggerDisplay(LOG4Z_MAIN_LOGGER_ID, false);
    size_t num_params = count_parameters(_model);
    LOGFMTI("Model has %lu parameters.", num_params);
    // 打印层数和参数维度
    print_model_summary(_model);
    zsummer::log4z::ILog4zManager::getRef().setLoggerDisplay(LOG4Z_MAIN_LOGGER_ID, true);
}

void HumanoidPolicy::update_History(const std::vector<float>& obs, const std::vector<float>& action) {
    // check input observation size & action size
    if (obs.size() != _obs_dim) {
        throw std::invalid_argument("Observation size mismatch: expected " + std::to_string(_obs_dim) + ", got " + std::to_string(obs.size()));
    }
    if (action.size() != _action_dim) {
        throw std::invalid_argument("Action size mismatch: expected " + std::to_string(_action_dim) + ", got " + std::to_string(action.size()));
    }
    // update observation history
    _obs_history.push_back(obs);
    if (_obs_history.size() >= _history_length) {
        _obs_history.pop_front();
    }

    // update last_action
    _last_action = action;
    // LOGI("Updated history with observation and action.");
    return;    
}