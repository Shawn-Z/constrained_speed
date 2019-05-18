#ifndef SJERK_HPP
#define SJERK_HPP

#include <vector>
#include <stdint.h>
#include <cmath>
#include <stdlib.h>
#include "SVector.hpp"

namespace shawn {

class SJerk {
public:
    SVector s_vector;

    /**
     * @brief calculate acceleration for @param p_v1 to @param p_v2
     * @param p_v1 from this speed
     * @param p_v2 to this speed
     * @param p_interval the interval between two speeds
     * @return signed
     */
    inline double_t calculate_acc(const double_t p_v1, const double_t p_v2, const double_t p_interval) {
        return ((pow(p_v2, 2.0) - pow(p_v1, 2.0)) * 0.5 / p_interval);
    }

    /**
     * @brief calculate speed of next point based on current point
     * @param p_v0 the speed of base point
     * @param p_a0 the acceleration of base point, (if less than zero, set to zero)
     * @param p_interval the interval between two points
     * @param p_jerk_max positive, describe how fast the speed changes
     * @param p_a_max limit max acceleration between base point and next point
     * @param p_a_min limit min acceleration between base point and next point
     * @param p_convex try to slow down if true, and try to speed up if false
     * @return the speed of next point, remember to limit the range in the next program
     */
    double_t calculate_next_speed(double_t p_v0, double_t p_a0, double_t p_interval, double_t p_jerk_max,
                                  double_t p_a_max, double_t p_a_min, bool p_convex) {
        p_jerk_max = fabs(p_jerk_max);
        double_t v1_square, a1;
        if (p_convex) {
            a1 = p_a0 - p_jerk_max * p_interval / std::max(p_v0, 0.01);
        } else {
            a1 = p_a0 + p_jerk_max * p_interval / std::max(p_v0, 0.01);
        }
        if (a1 > p_a_max) {a1 = p_a_max;}
        if (a1 < p_a_min) {a1 = p_a_min;}
        v1_square = pow(p_v0, 2.0) + 2.0 * a1 * p_interval;
        if (v1_square <= 0) {
            return 0;
        }
        return sqrt(v1_square);
    }

    /**
     * @brief calculate previous speed based on current point
     * @param p_v1 the speed of base point
     * @param p_a1 the acceleration of base point
     * @param p_interval the interval between two points
     * @param p_jerk_max positive, describe how fast the speed changes
     * @param p_a_max limit max acceleration between base point and previous point, (if more than zero, set to zero)
     * @param p_a_min limit min acceleration between base point and previous point
     * @param p_convex try to slow down if true, and try to speed up if false
     * @param p_remove_dec while the speed of base point more than the speed of first point, limit max deceleration(min acceleration) or not
     * @param p_remove_dec_value no more than zero, if set @param p_remove_dec true, set the max deceleration(min acceleration) while condition satisfied
     * @param p_v_begin the first speed in the sequence, as the current speed
     * @return the speed of previous point, remember to limit the range in the next program
     */
    double_t calculate_previous_speed(double_t p_v1, double_t p_a1, double_t p_interval, double_t p_jerk_max,
                                      double_t p_a_max, double_t p_a_min, bool p_convex,
                                      bool p_remove_dec = false, double_t p_remove_dec_value = 0, double_t p_v_begin = 0) {
        p_jerk_max = fabs(p_jerk_max);
        double_t v0_square, a0;
        if (p_convex) {
            a0 = p_a1 + p_jerk_max * p_interval / std::max(p_v1, 0.01);
        } else {
            a0 = p_a1 - p_jerk_max * p_interval / std::max(p_v1, 0.01);
        }
        if (a0 > p_a_max) {a0 = p_a_max;}
        if (a0 < p_a_min) {a0 = p_a_min;}
        if (p_remove_dec) {
            if (p_v1 > p_v_begin) {
                if (a0 < p_remove_dec_value) {
                    a0 = p_remove_dec_value;
                }
            }
        }
        v0_square = pow(p_v1, 2.0) - 2.0 * a0 * p_interval;
        if (v0_square <= 0) {
            return 0;
        }
        return sqrt(v0_square);
    }

    /**
     * @brief calculate current point speed satisfy jerk condition
     * @details the result speed together with neighboring speeds forms a convex shape
     * @param p_former_v the speed of previous point
     * @param p_latter_v the speed of next point
     * @param p_former_interval the interval between current point and previous point
     * @param p_latter_interval the interval between current point and next point
     * @param p_jerk_max positive. describe how fast the speed changes
     * @param p_convex smooth as convex or not
     * @return the speed of current point
     */
    double_t calculate_middle_speed(double_t p_former_v, double_t p_latter_v,
                                    double_t p_former_interval, double_t p_latter_interval,
                                    double_t p_jerk_max, bool p_convex) {
        p_jerk_max = fabs(p_jerk_max);
        double_t v_square;
        if (p_convex) {
            v_square = (p_former_interval * pow(p_latter_v, 2.0) + p_latter_interval * pow(p_former_v, 2.0)) /
                       (p_former_interval + p_latter_interval) +
                       4.0 * p_jerk_max * p_former_interval * p_latter_interval / (p_former_v + p_latter_v);
        } else {
            v_square = (p_former_interval * pow(p_latter_v, 2.0) + p_latter_interval * pow(p_former_v, 2.0)) /
                       (p_former_interval + p_latter_interval) -
                       4.0 * p_jerk_max * p_former_interval * p_latter_interval / (p_former_v + p_latter_v);
        }
        if (v_square <= 0) {
            return 0;
        }
        return sqrt(v_square);
    }

    /**
     * @brief smooth speed in forward pass
     * @details while satisfy max speed sequence and jerk, try to increase speed along arc-length
     * @param p_v input speed sequence
     * @param p_interval input interval sequence
     * @param p_jerk_max condition of max jerk, positive
     * @param p_a_max max acceleration, signed
     * @param p_a_min min acceleration, signed
     * @param p_a_begin the acceleration of first point, as current acceleration
     */
    void forward_pass_concave(std::vector<double_t> &p_v, std::vector<double_t> &p_interval,
                      double_t p_jerk_max, double_t p_a_max, double_t p_a_min, double_t p_a_begin) {
        p_jerk_max = fabs(p_jerk_max);
        size_t tmp_size = std::min(p_v.size(), p_interval.size());
        if (tmp_size < 2) {
            return;
        }
        double_t tmp_speed;
        double_t tmp_acc;
        tmp_acc = std::max(p_a_begin, 0.0); /// @see calculate_next_speed make p_a0 at least zero
        tmp_speed = calculate_next_speed(p_v[0], tmp_acc, p_interval[1], p_jerk_max, p_a_max, p_a_min, false);
        p_v[1] = std::min(p_v[1], tmp_speed);
        for (size_t i = 2; i < tmp_size; ++i) {
            tmp_acc = std::max(calculate_acc(p_v[i - 2], p_v[i - 1], p_interval[i - 1]), 0.0); /// @see calculate_next_speed make p_a0 at least zero
            tmp_speed = calculate_next_speed(p_v[i - 1], tmp_acc, p_interval[i], p_jerk_max, p_a_max, p_a_min, false);
            p_v[i] = std::min(p_v[i], tmp_speed);
        }
    }

    /**
     * @brief smooth speed in backward pass
     * @param p_v input speed sequence
     * @param p_interval input interval sequence
     * @param p_jerk_max condition of max jerk, positive
     * @param p_a_max max acceleration, signed
     * @param p_a_min min acceleration, signed
     * @param p_a_end no more than zero, describe the deceleration of close to the end point
     * @param p_v_begin_threshold determine if the speed curve can be generate
     * @param p_remove_dec while the speed of base point more than the speed of first point, limit max deceleration(min acceleration) or not
     * @param p_remove_dec_value no more than zero, if set @param p_remove_dec true, set the max deceleration(min acceleration) while condition satisfied
     * @param p_v_begin the first speed in the sequence, as the current speed
     * @return true if the speed curve can be generate
     */
    bool backward_pass_concave(std::vector<double_t> &p_v, std::vector<double_t> &p_interval,
                       double_t p_jerk_max, double_t p_a_max, double_t p_a_min, double_t p_a_end, double_t p_v_begin_threshold,
                       bool p_remove_dec = false, double_t p_remove_dec_value = 0, double_t p_v_begin = 0) {
        p_jerk_max = fabs(p_jerk_max);
        p_remove_dec_value = -fabs(p_remove_dec_value);
        size_t tmp_size = std::min(p_v.size(), p_interval.size());
        if (tmp_size < 3) {
            return false;
        }
        p_a_max = std::min(p_a_max, 0.0); /// @see calculate_previous_speed make p_a_max no more than zero
        double_t tmp_speed;
        double_t tmp_acc;
        tmp_acc = -fabs(p_a_end);
        tmp_speed = calculate_previous_speed(p_v[tmp_size - 1], tmp_acc, p_interval[tmp_size - 1],
                                             p_jerk_max, p_a_max, p_a_min,
                                             false, p_remove_dec, p_remove_dec_value, p_v_begin);
        p_v[tmp_size - 2] = std::min(p_v[tmp_size - 2], tmp_speed);


        for (size_t i = tmp_size - 3; i > 0; --i) {
            tmp_acc = calculate_acc(p_v[i + 1], p_v[i + 2], p_interval[i + 2]);
            tmp_speed = calculate_previous_speed(p_v[i + 1], tmp_acc, p_interval[i + 1],
                                                 p_jerk_max, p_a_max, p_a_min,
                                                 false, p_remove_dec, p_remove_dec_value, p_v_begin);
            p_v[i] = std::min(p_v[i], tmp_speed);
        }
        tmp_acc = calculate_acc(p_v[1], p_v[2], p_interval[2]);
        tmp_speed = calculate_previous_speed(p_v[1], tmp_acc, p_interval[1],
                                             p_jerk_max, p_a_max, p_a_min,
                                             false, p_remove_dec, p_remove_dec_value, p_v_begin);
        return ((p_v[0] - tmp_speed) < fabs(p_v_begin_threshold));
    }

    /**
     * @brief adjust the middle point of three points
     * @param p_v input speed sequence
     * @param p_interval input interval sequence
     * @param p_jerk_max condition of max jerk, positive
     */
    void middle_pass_convex(std::vector<double_t> &p_v, std::vector<double_t> &p_interval, double_t p_jerk_max) {
        p_jerk_max = fabs(p_jerk_max);
        size_t tmp_size = std::min(p_v.size(), p_interval.size());
        if (tmp_size < 3) {
            return;
        }
        for (size_t i = 1; i < tmp_size - 1; ++i) {
            p_v[i] = std::min(p_v[i], calculate_middle_speed(p_v[i - 1], p_v[i + 1], p_interval[i], p_interval[i + 1], p_jerk_max, true));
        }
    }

    /**
     * @brief cycle the function @see middle_pass_cycle to smooth the speed curve
     * @param p_v input speed sequence
     * @param p_interval input interval sequence
     * @param p_jerk_max condition of max jerk, positive
     * @param p_max_diff max difference between two cycle
     * @param p_max_cycles max cycle times
     */
    void middle_pass_convex_cycle(std::vector<double_t> &p_v, std::vector<double_t> &p_interval, double_t p_jerk_max,
                                  double_t p_max_diff, size_t p_max_cycles) {
        p_jerk_max = fabs(p_jerk_max);
        std::vector<double_t> v_history;
        do {
            v_history = p_v;
            middle_pass_convex(p_v, p_interval, p_jerk_max);
        } while ((--p_max_cycles) && (this->s_vector.get_max_absolute_diff<double_t>(p_v, v_history) > p_max_diff));
    }

    /**
     * @brief if current speed more than max speed set, generate a speed curve smooth slow down to max speed set
     * @param p_v input speed sequence
     * @param p_interval input interval sequence
     * @param p_goal max speed set
     * @param p_a_begin current speed
     * @param p_a_max max acceleration
     * @param p_a_min min acceleration
     * @param p_jerk_max_forward condition of max jerk in forward pass, positive
     * @param p_jerk_max_middle condition of max jerk in middle pass, positive
     * @param p_max_cycles max difference between two cycle for middle pass
     * @param p_max_diff max cycle times for middle pass
     */
    void smooth_slow_down(std::vector<double_t> &p_v, std::vector<double_t> p_interval, double_t p_goal,
                          double_t p_a_begin, double_t p_a_max, double_t p_a_min,
                          double_t p_jerk_max_forward, double_t p_jerk_max_middle,
                          double_t p_max_diff, size_t p_max_cycles) {
        p_jerk_max_forward = fabs(p_jerk_max_forward);
        p_jerk_max_middle = fabs(p_jerk_max_middle);
        size_t tmp_size = std::min(p_v.size(), p_interval.size());
        if (tmp_size < 3) {
            return;
        }
        if (p_v[0] <= p_goal) {
            return;
        }
        double_t tmp_speed;
        double_t tmp_acc;
        tmp_acc = std::min(p_a_begin, 0.0); /// make p_a0 at more zero
        tmp_speed = calculate_next_speed(p_v[0], tmp_acc, p_interval[1], p_jerk_max_forward, p_a_max, p_a_min, true);
        p_v[1] = std::max(p_goal, tmp_speed);
        for (size_t i = 2; i < tmp_size; ++i) {
            tmp_acc = std::min(calculate_acc(p_v[i - 2], p_v[i - 1], p_interval[i - 1]), 0.0); /// @see calculate_next_speed make p_a0 at least zero
            tmp_speed = calculate_next_speed(p_v[i - 1], tmp_acc, p_interval[i], p_jerk_max_forward, p_a_max, p_a_min, true);
            p_v[i] = std::max(p_goal, tmp_speed);
        }
        std::vector<double_t> v_history;
        do {
            v_history = p_v;
            for (size_t i = 1; i < tmp_size - 1; ++i) {
                p_v[i] = std::max(p_v[i], calculate_middle_speed(p_v[i - 1], p_v[i + 1], p_interval[i - 1], p_interval[i + 1], p_jerk_max_middle, false));
            }
        } while ((--p_max_cycles) && (this->s_vector.get_max_absolute_diff<double_t>(p_v, v_history) > p_max_diff));
    }
};
    
}

#endif //SJERK_HPP