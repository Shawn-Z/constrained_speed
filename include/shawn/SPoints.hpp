#ifndef SHAWNPOINTS_HPP
#define SHAWNPOINTS_HPP

#include <vector>
#include <stdint.h>
#include <cmath>
#include <stdlib.h>

namespace shawn {

class SPoints {
public:
    inline double_t get_two_point_interval(const double_t &p_x1, const double_t &p_y1,
                                           const double_t &p_x2, const double_t &p_y2) {
        return pow(pow((p_x1 - p_x2), 2.0) + pow((p_y1 - p_y2), 2.0), 0.5);
    }

    std::vector<std::vector<double_t>> calculate_intervals_arc_lengths(const std::vector<double_t> &p_x_points, const std::vector<double_t> &p_y_points) {
        std::vector<std::vector<double_t>> intervals_arc_lengths;
        intervals_arc_lengths.clear();
        size_t points_size = p_x_points.size();
        if (points_size < 2) {
            return intervals_arc_lengths;
        }
        if (points_size != p_y_points.size()) {
            return intervals_arc_lengths;
        }
        std::vector<double_t> intervals;
        std::vector<double_t> arc_lengths;
        intervals.clear();
        arc_lengths.clear();
        intervals.emplace_back(0);
        arc_lengths.emplace_back(0);
        for (size_t i = 1; i < points_size; ++i) {
            intervals.emplace_back(get_two_point_interval(p_x_points[i - 1], p_y_points[i - 1],
                                                          p_x_points[i], p_y_points[i]));
            arc_lengths.emplace_back(arc_lengths[i - 1] + intervals[i]);
        }
        intervals_arc_lengths.emplace_back(intervals);
        intervals_arc_lengths.emplace_back(arc_lengths);
        return intervals_arc_lengths;
    }
};

}

#endif //SHAWNPOINTS_HPP