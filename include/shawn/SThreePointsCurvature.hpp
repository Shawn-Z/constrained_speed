#ifndef STHREEPOINTSCUVATURE_HPP
#define STHREEPOINTSCUVATURE_HPP

#include <vector>
#include <stdint.h>
#include <cmath>
#include <stdlib.h>

namespace shawn {

class SThreePointsCurvature {
public:
    std::vector<std::vector<double_t>> findCurvatureCalculatePoints(const std::vector<double_t> &p_x_points,
                                                                    const std::vector<double_t> &p_y_points,
                                                                    const std::vector<double_t> &p_arc_lengths,
                                                                    const std::vector<double_t> &p_intervals,
                                                                    double_t p_waist_length,
                                                                    size_t &p_p_start_index,
                                                                    size_t &p_p_end_index) {
        std::vector<std::vector<double_t>> retrieve;
        retrieve.clear();
        size_t points_size = p_x_points.size();
        double_t total_length = p_arc_lengths.back();
        if (points_size < 3) {
            return retrieve;
        }
        if (points_size != p_y_points.size()) {
            return retrieve;
        }
        if (points_size != p_arc_lengths.size()) {
            return retrieve;
        }
        if (points_size != p_intervals.size()) {
            return retrieve;
        }
        if (p_waist_length <= 0) {
            return retrieve;
        }
        if (total_length < 2.0 * p_waist_length) {
            return retrieve;
        }

        size_t start_index = points_size;
        size_t end_index = 0;
        for (size_t i = 1; i < points_size; ++i) {
            if (p_arc_lengths[i] > p_waist_length) {
                start_index = i;
                break;
            }
        }
        for (size_t j = points_size - 1; j >= 0; --j) {
            if (total_length - p_arc_lengths[j] > p_waist_length) {
                end_index = j;
                break;
            }

        }
        if (start_index >= end_index) {
            return retrieve;
        }

        double_t tmp_ratio;
        std::vector<double_t> former_x_points(points_size, 0);
        std::vector<double_t> former_y_points(points_size, 0);
        std::vector<double_t> latter_x_points(points_size, 0);
        std::vector<double_t> latter_y_points(points_size, 0);
        for (size_t k = start_index; k <= end_index; ++k) {
            for (size_t m = k - 1; m >= 0; --m) {
                if (p_arc_lengths[k] - p_arc_lengths[m] > p_waist_length) {
                    tmp_ratio = (p_arc_lengths[k] - p_arc_lengths[m] - p_waist_length) / p_intervals[m + 1];
                    former_x_points[k] = p_x_points[m] + tmp_ratio * (p_x_points[m + 1] - p_x_points[m]);
                    former_y_points[k] = p_y_points[m] + tmp_ratio * (p_y_points[m + 1] - p_y_points[m]);
                    break;
                }
            }
            for (size_t n = k + 1; n < points_size; ++n) {
                if (p_arc_lengths[n] - p_arc_lengths[k] > p_waist_length) {
                    tmp_ratio = (p_waist_length - (p_arc_lengths[n - 1] - p_arc_lengths[k])) / p_intervals[n];
                    latter_x_points[k] = p_x_points[n - 1] + tmp_ratio * (p_x_points[n] - p_x_points[n - 1]);
                    latter_y_points[k] = p_y_points[n - 1] + tmp_ratio * (p_y_points[n] - p_y_points[n - 1]);
                    break;
                }
            }
        }
        retrieve.emplace_back(former_x_points);
        retrieve.emplace_back(former_y_points);
        retrieve.emplace_back(latter_x_points);
        retrieve.emplace_back(latter_y_points);
        p_p_start_index = start_index;
        p_p_end_index = end_index;
        return retrieve;
    }

    double_t calculatePointUnsignedCurvature(double_t p_former_x, double_t p_former_y,
                                             double_t p_latter_x, double_t p_latter_y, double_t p_waist_length) {
        double_t bottom_length_square = pow((p_former_x - p_latter_x), 2.0) + pow((p_former_y - p_latter_y), 2.0);
        double_t curv = 2 * sqrt(fabs(pow(p_waist_length, 2.0) - 0.25 * bottom_length_square)) / pow(p_waist_length, 2.0);
        return curv;
    }

    std::vector<double_t> calculateUnsignedCurvatures(const std::vector<double_t> &p_x_points,
                                                      const std::vector<double_t> &p_y_points,
                                                      const std::vector<double_t> &p_arc_lengths,
                                                      const std::vector<double_t> &p_intervals,
                                                      double_t p_waist_length) {
        std::vector<double_t> curvatures;
        curvatures.clear();
        std::vector<std::vector<double_t>> calculate_points;
        size_t start_index;
        size_t end_index;
        calculate_points = findCurvatureCalculatePoints(p_x_points, p_y_points, p_arc_lengths, p_intervals,
                                                        p_waist_length, start_index, end_index);
        if (calculate_points.empty()) {
            return curvatures;
        }
        size_t points_size = p_x_points.size();
        if (points_size < 3) {
            return curvatures;
        }
        curvatures.resize(points_size);
        std::vector<double_t> &former_x_points = calculate_points[0];
        std::vector<double_t> &former_y_points = calculate_points[1];
        std::vector<double_t> &latter_x_points = calculate_points[2];
        std::vector<double_t> &latter_y_points = calculate_points[3];
        for (size_t i = start_index; i <= end_index; ++i) {
            curvatures[i] = calculatePointUnsignedCurvature(former_x_points[i], former_y_points[i],
                                                            latter_x_points[i], latter_y_points[i], p_waist_length);
        }
        for (size_t j = 0; j < start_index; ++j) {
            curvatures[j] = curvatures[start_index];
        }
        for (size_t k = end_index + 1; k < points_size; ++k) {
            curvatures[k] = curvatures[end_index];
        }
    }

    void curvaturesReproduce(std::vector<double_t> &p_curvatures, double_t p_concerned_curvature) {
        if (p_concerned_curvature <= 0) {
            return;
        }
        double_t tmp_max_curvature;
        size_t points_size = p_curvatures.size();
        for (size_t i = 0; i < points_size; ++i) {
            if (fabs(p_curvatures[i]) > p_concerned_curvature) {
                tmp_max_curvature = 0;
                for (size_t j = i; j < points_size; ++j) {
                    if (fabs(p_curvatures[j]) <= p_concerned_curvature) {
                        break;
                    }
                    if (tmp_max_curvature < fabs(p_curvatures[j])) {
                        tmp_max_curvature = fabs(p_curvatures[j]);
                    }
                    i = j;
                }
                for (size_t k = i; k >= 0; --k) {
                    if (fabs(p_curvatures[k]) <= p_concerned_curvature) {
                        break;
                    }
                    p_curvatures[k] = tmp_max_curvature;
                }
            }
        }
    }

    void absoluteCurvaturesLimited(std::vector<double_t> &p_curvatures, double_t p_max_unsigned_curvature) {
        if (p_max_unsigned_curvature <= 0) {
            return;
        }
        size_t points_size = p_curvatures.size();
        for (size_t i = 0; i < points_size; ++i) {
            if (fabs(p_curvatures[i]) > p_max_unsigned_curvature) {
                p_curvatures[i] = (p_curvatures[i] > 0? p_max_unsigned_curvature: (-p_max_unsigned_curvature));
            }
        }
    }

    std::vector<double_t> speedPlanningCurvature(const std::vector<double_t> &p_x_points,
                                                 const std::vector<double_t> &p_y_points,
                                                 const std::vector<double_t> &p_arc_lengths,
                                                 const std::vector<double_t> &p_intervals,
                                                 double_t p_waist_length,
                                                 double_t p_concerned_curvature,
                                                 double_t p_max_unsigned_curvature) {
        std::vector<double_t> curvatures;
        curvatures = calculateUnsignedCurvatures(p_x_points, p_y_points, p_arc_lengths, p_intervals, p_waist_length);
        curvaturesReproduce(curvatures, p_concerned_curvature);
        absoluteCurvaturesLimited(curvatures, p_max_unsigned_curvature);
        return curvatures;
    }
};

}

#endif //STHREEPOINTSCUVATURE_HPP