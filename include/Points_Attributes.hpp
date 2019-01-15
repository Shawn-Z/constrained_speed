#ifndef CONSTRAINED_SPEED_POINTS_ATTRIBUTES_HPP
#define CONSTRAINED_SPEED_POINTS_ATTRIBUTES_HPP

#include <vector>
#include <cmath>
#include <stdint.h>
#include <iostream>
#include <ros/ros.h>

namespace points_attributes {

typedef int16_t count_type;
typedef double_t cal_type;

struct params_type {
    cal_type half_s_curv_cal;
    cal_type curv_zero_redundancy;
    cal_type curvature_max;
    count_type min_size;
    count_type max_size;
    cal_type min_total_length;
    //// todo add max total length
};

struct error_mark_type {
    union {
        struct {
            bool system_size_over: 1;
            bool points_num_unequal: 1;
            bool points_num_less: 1;
            bool points_num_over: 1;
            bool points_empty: 1;
            bool s_damn_short: 1;
        } items;
        uint8_t result;
    } update;
};

struct warn_mark_type {
    union {
        struct {
            bool curv_index_warn: 1;
            bool short_for_curv: 1;
        } items;
        uint8_t result;
    } update;
};

struct curv_cal_type {
    count_type start_curv_index;
    count_type end_curv_index;
    cal_type ratio;
    std::vector<cal_type> curv_one_x;
    std::vector<cal_type> curv_one_y;
    std::vector<cal_type> curv_thr_x;
    std::vector<cal_type> curv_thr_y;

    cal_type tmp_point_curv;
    cal_type curvature_max;
    std::vector<cal_type> curv_origin;
    std::vector<bool> curv_zero_sign;
    std::vector<cal_type> curv_limited;
    std::vector<cal_type> curv_final;
};

class Points_Attributes {

public:
    bool fully_run;

    params_type params;

    error_mark_type error;
    warn_mark_type warn;

    count_type x_size;
    count_type y_size;

    std::vector<cal_type> s;
    std::vector<cal_type> interval;

    curv_cal_type curv_cal;


    void calculate(const params_type tmp_params, const std::vector<cal_type> & x_points, const std::vector<cal_type> & y_points);

private:
    void state_reset_before();
    void pretreat(const std::vector<cal_type> & x_points, const std::vector<cal_type> & y_points);
    void get_arc_length(const std::vector<cal_type> & x_points, const std::vector<cal_type> & y_points);
    void get_curvature(const std::vector<cal_type> & x_points, const std::vector<cal_type> & y_points);
    void find_curv_index(const std::vector<cal_type> & x_points, const std::vector<cal_type> & y_points);
    cal_type get_point_curv(const count_type i, const std::vector<cal_type> & x_points, const std::vector<cal_type> & y_points);
    void curv_reproduce();
};

}

#endif //CONSTRAINED_SPEED_POINTS_ATTRIBUTES_HPP