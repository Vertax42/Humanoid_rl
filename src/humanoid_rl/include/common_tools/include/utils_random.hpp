#ifndef __UTILS_RANDOM_HPP__
#define __UTILS_RANDOM_HPP__

#include <algorithm>
#include <iostream>
#include <random>
#include <stdio.h>
#include <stdlib.h>

#include <algorithm>
#include <iostream>
#include <random>
#include <stdio.h>
#include <stdlib.h>

namespace common_tools {
template <typename T>
struct Random_generator_float {
    std::random_device random_device;
    std::mt19937 m_random_engine;
    std::uniform_real_distribution<T> m_dist;
    std::normal_distribution<T> m_dist_normal;
    Random_generator_float(bool if_true_random = true, int seed = 0)
    {
        if(if_true_random)
        {
            m_random_engine = std::mt19937(std::random_device{}());
        } else
        {
            m_random_engine = std::mt19937(seed);
        }
    }
    ~Random_generator_float(){};

    T rand_uniform(T low = 0.0, T hight = 1.0)
    {
        m_dist = std::uniform_real_distribution<T>(low, hight);
        return m_dist(m_random_engine);
    }

    T rand_normal(T mean = 0.0, T std = 100.0)
    {
        m_dist_normal = std::normal_distribution<T>(mean, std);
        return m_dist_normal(m_random_engine);
    }

    T *rand_array_uniform(T low = 0.0, T hight = 1.0, size_t numbers = 100, T *res = nullptr)
    {
        if(res == nullptr)
        {
            res = new T[numbers];
        }
        m_dist = std::uniform_real_distribution<T>(low, hight);
        for(size_t i = 0; i < numbers; i++)
        {
            res[i] = m_dist(m_random_engine);
        }
        return res;
    }

    T *rand_array_normal(T mean = 0.0, T std = 1.0, size_t numbers = 100, T *res = nullptr)
    {
        if(res == nullptr)
        {
            res = new T[numbers];
        }
        m_dist_normal = std::normal_distribution<T>(mean, std);
        for(size_t i = 0; i < numbers; i++)
        {
            res[i] = m_dist_normal(m_random_engine);
        }
        return res;
    }
};

template <typename T>
struct Random_generator_int {
    std::random_device random_device;
    std::mt19937 m_random_engine;
    std::uniform_int_distribution<T> m_dist;
    Random_generator_int(bool if_true_random = true, int seed = 0)
    {
        if(if_true_random)
        {
            m_random_engine = std::mt19937(std::random_device{}());
        } else
        {
            m_random_engine = std::mt19937(seed);
        }
    }

    ~Random_generator_int(){};

    T rand_uniform(T low = 0, T hight = 100)
    {
        m_dist = std::uniform_int_distribution<T>(low, hight);
        return m_dist(m_random_engine);
    }

    T *rand_array_uniform(T low = 0.0, T hight = 1.0, size_t numbers = 100, T *res = nullptr)
    {
        if(res == nullptr)
        {
            res = new T[numbers];
        }
        m_dist = std::uniform_int_distribution<T>(low, hight);
        for(size_t i = 0; i < numbers; i++)
        {
            res[i] = m_dist(m_random_engine);
        }
        return res;
    }

    T *rand_array_norepeat(T low, T high, T k)
    {
        T n = high - low;
        T *res_array = new T[k];
        std::vector<int> foo;
        foo.resize(n);
        for(T i = 1; i <= n; ++i)
            foo[i] = i + low;
        std::shuffle(foo.begin(), foo.end(), m_random_engine);
        for(T i = 0; i < k; ++i)
        {
            res_array[i] = foo[i];
            // std::cout << foo[ i ] << " ";
        }
        return res_array;
    }
};

} // namespace common_tools

#endif // UTILS_RANDOM