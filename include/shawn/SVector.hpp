#ifndef SHAWN_VECTOR_HPP
#define SHAWN_VECTOR_HPP

#include <vector>
#include <cmath>
#include <stdlib.h>

namespace shawn {

class SVector {
public:
    template <typename T1, typename T2>
    void unifySize(std::vector<T1> &p_vector1, std::vector<T2> &p_vector2) {
        if (p_vector1.size() == p_vector2.size()) {
            return;
        }
        if (p_vector1.size() > p_vector2.size()) {
            p_vector2.resize(p_vector1.size());
        } else {
            p_vector1.resize(p_vector2.size());
        }
    }

    template <typename T1>
    void unifySize(std::vector<T1> &p_vector1, size_t p_size) {
        if (p_vector1.size() < p_size) {
            p_vector1.resize(p_size);
        }
    }

    template <typename T1, typename T2>
    bool checkSizeSame(std::vector<T1> &p_vector1, std::vector<T2> &p_vector2) {
        return (p_vector1.size() == p_vector2.size());
    }

    template <typename T1, typename T2, typename T3>
    T1 get_max_absolute_diff(const std::vector<T2> p_V1, const std::vector<T3> p_V2) {
        size_t tmp_size = std::min(p_V1.size(), p_V2.size());
        if (tmp_size < 1) {
            return 0;
        }
        T1 max_absolute_diff = fabs(p_V1[0] - p_V2[0]);
        T1 tmp_diff = max_absolute_diff;
        for (size_t i = 1; i < tmp_size; ++i) {
            tmp_diff = fabs(p_V1[i] - p_V2[i]);
            if (max_absolute_diff < tmp_diff) {
                max_absolute_diff = tmp_diff;
            }
        }
        return max_absolute_diff;
    }
};

}

#endif //SHAWN_VECTOR_HPP