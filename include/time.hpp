/* Copyright (c) 2017, Yu Zhang, Intelligent Vehicle Research Center(IVRC),
 * Beijing Institute of Technology.
 *        Files: time.hpp
 *   Created on: 05, 11, 2017
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */
#ifndef OPT_UTILS_TIME_HPP
#define OPT_UTILS_TIME_HPP
#include <chrono>
namespace hmpl {

inline double getDurationInSecs(
        const std::chrono::time_point<std::chrono::steady_clock> &start,
        const std::chrono::time_point<std::chrono::steady_clock> &end) {
    double elapsed_secondes =
            std::chrono::duration_cast<std::chrono::duration<double>>(end -
                                                                      start)
                    .count();
    return elapsed_secondes;
}

inline std::chrono::time_point<std::chrono::steady_clock> now() { return std::chrono::steady_clock::now();}

}  // namespace hmpl
#endif  // OPT_UTILS_TIME_HPP
