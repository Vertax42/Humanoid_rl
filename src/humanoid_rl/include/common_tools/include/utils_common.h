#ifndef __UTILS_COMMON_HPP__
#define __UTILS_COMMON_HPP__

#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <cstdlib>
#include <future>
#include <list>
#include <set>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <vector>

#include "log4z.h"
#include "utils_logger.hpp"
#include "utils_os_compatible.hpp"
#include "utils_random.hpp"
#include "utils_timer.hpp"

namespace common_tools {
// system related tools
void dump_program_info_log4z(const std::string &app_name);

void clean_log_files(int max_log_count);

// boost::filesystem related tools
std::string get_currentpath(const std::string &path);

std::string get_filename(const std::string &path);

long getFileSize(const std::string &filename);

// Eigen-related tools
std::string eigenMatrixToString(const Eigen::Matrix4f &mat);

template <typename T>
static std::vector<T> vector_2d_to_1d(std::vector<std::vector<T> > &pt_vec_vec)
{
    std::vector<T> pt_vec;
    for(unsigned int i = 0; i < pt_vec_vec.size(); i++)
    {
        pt_vec.insert(pt_vec.end(), pt_vec_vec[i].begin(), pt_vec_vec[i].end());
    }
    return pt_vec;
};

template <typename T>
std::vector<T> std_set_to_vector(const std::set<T> &input_set)
{
    std::vector<T> output_vector;
    for(auto it = input_set.begin(); it != input_set.end(); it++)
    {
        output_vector.push_back(*(it));
    }
    return output_vector;
};

template <typename T>
std::set<T> std_vector_to_set(const std::vector<T> &input_vector)
{
    std::set<T> output_set;
    for(auto it = input_vector.begin(); it != input_vector.end(); it++)
    {
        output_set.insert(*(it));
    }
    return output_set;
};

template <typename T>
void maintain_maximum_thread_pool(std::list<T> &thread_pool, size_t maximum_parallel_thread)
{
    if(thread_pool.size() >= (size_t)maximum_parallel_thread)
    {
        while(1)
        {
            for(auto it = thread_pool.begin(); it != thread_pool.end(); it++)
            {

                auto status = (*it)->wait_for(std::chrono::nanoseconds(1));
                if(status == std::future_status::ready)
                {
                    delete *it;
                    thread_pool.erase(it);
                    break;
                }
            }
            if(thread_pool.size() < (size_t)maximum_parallel_thread)
            {
                break;
            }
            std::this_thread::sleep_for(std::chrono::nanoseconds(1));
        }
    }
};

} // namespace common_tools

#endif // UTILS_COMMON_H