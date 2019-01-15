#include <Points_Attributes.hpp>

namespace points_attributes {

void Points_Attributes::calculate(const params_type tmp_params, const std::vector<cal_type> &x_points, const std::vector<cal_type> &y_points) {
    this->params = tmp_params;
    state_reset_before();
    pretreat(x_points, y_points);
    if (this->error.update.result != 0) {
        return;
    }
    get_arc_length(x_points, y_points);
    if (this->error.update.result != 0) {
        return;
    }
    get_curvature(x_points, y_points);
    this->fully_run = true;
}

void Points_Attributes::state_reset_before() {
    this->fully_run = false;

    this->error.update.result = 0;
    this->warn.update.result = 0;

    this->interval.clear();
    this->s.clear();

    this->curv_cal.curv_origin.clear();
    this->curv_cal.curv_zero_sign.clear();
    this->curv_cal.curv_limited.clear();
    this->curv_cal.curv_final.clear();

    this->curv_cal.curv_one_x.clear();
    this->curv_cal.curv_one_y.clear();
    this->curv_cal.curv_thr_x.clear();
    this->curv_cal.curv_thr_y.clear();
}

void Points_Attributes::pretreat(const std::vector<cal_type> &x_points, const std::vector<cal_type> & y_points) {
    if ((x_points.size() > UINT32_MAX) || (y_points.size() > UINT32_MAX)) {
        this->error.update.items.system_size_over = true;
        return;
    }
    this->x_size = x_points.size();
    this->y_size = y_points.size();
    if ((this->x_size == 0) || (this->y_size == 0)) {
        this->error.update.items.points_empty = true;
    }
    if (this->x_size != this->y_size) {
        this->error.update.items.points_num_unequal = true;
        return;
    }
    if ((this->x_size < this->params.min_size) || (this->x_size < 3)) {
        this->error.update.items.points_num_less = true;
    }
    if (this->x_size > this->params.max_size) {
        this->error.update.items.points_num_over = true;
    }
}

void Points_Attributes::get_arc_length(const std::vector<cal_type> &x_points, const std::vector<cal_type> &y_points) {
    this->interval.emplace_back(0);
    this->s.emplace_back(0);
    for (count_type i = 1; i < this->x_size; ++i) {
        this->interval.emplace_back(sqrt(pow((x_points[i] - x_points[i - 1]), 2.0) + pow((y_points[i] - y_points[i - 1]), 2.0)));
        this->s.emplace_back(this->s[i - 1] + this->interval[i]);
    }
    if (this->s.back() < this->params.min_total_length) {
        this->error.update.items.s_damn_short = true;
    }
    if (this->s.back() < (2.1 * this->params.half_s_curv_cal)) {
        this->warn.update.items.short_for_curv = true;
    }
}

void Points_Attributes::get_curvature(const std::vector<cal_type> &x_points, const std::vector<cal_type> &y_points) {
    this->curv_cal.curv_origin.assign(this->x_size, this->params.curvature_max);
    this->curv_cal.curv_zero_sign.assign(this->x_size, false);
    this->curv_cal.curv_limited.assign(this->x_size, this->params.curvature_max);
    this->curv_cal.curv_final.assign(this->x_size, this->params.curvature_max);

    if (this->warn.update.items.short_for_curv) {
        return;
    }

    find_curv_index(x_points, y_points);
    if (this->warn.update.items.curv_index_warn) {
        return;
    }

    for (count_type i = this->curv_cal.start_curv_index; i <= this->curv_cal.end_curv_index; ++i) {
        this->curv_cal.tmp_point_curv = get_point_curv(i, x_points, y_points);
        this->curv_cal.curv_origin[i] = this->curv_cal.tmp_point_curv;
        this->curv_cal.curv_zero_sign[i] = (this->curv_cal.tmp_point_curv <= this->params.curv_zero_redundancy);
        this->curv_cal.curv_limited[i] = std::min(this->curv_cal.tmp_point_curv, this->params.curvature_max);
    }

    for (count_type i = 0; i < this->curv_cal.start_curv_index; ++i) {
        this->curv_cal.curv_origin[i] = this->curv_cal.curv_origin[this->curv_cal.start_curv_index];
        this->curv_cal.curv_zero_sign[i] = this->curv_cal.curv_zero_sign[this->curv_cal.start_curv_index];
        this->curv_cal.curv_limited[i] = this->curv_cal.curv_limited[this->curv_cal.start_curv_index];
    }
    for (count_type i = (this->curv_cal.end_curv_index + 1); i < this->x_size; ++i) {
        this->curv_cal.curv_origin[i] = this->curv_cal.curv_origin[this->curv_cal.end_curv_index];
        this->curv_cal.curv_zero_sign[i] = this->curv_cal.curv_zero_sign[this->curv_cal.end_curv_index];
        this->curv_cal.curv_limited[i] = this->curv_cal.curv_limited[this->curv_cal.end_curv_index];
    }
    curv_reproduce();
    this->curv_cal.curv_final = this->curv_cal.curv_limited;

    this->curv_cal.curvature_max = this->curv_cal.curv_final[0];
    for (auto & tmp : this->curv_cal.curv_final) {
        if (this->curv_cal.curvature_max < tmp) {
            this->curv_cal.curvature_max = tmp;
        }
    }
}

void Points_Attributes::find_curv_index(const std::vector<cal_type> &x_points, const std::vector<cal_type> &y_points) {
    this->curv_cal.start_curv_index = 0;
    this->curv_cal.end_curv_index = x_size;
    for (count_type i = 1; i < this->x_size; ++i) {
        if ((this->s[i] - this->s[0]) >= this->params.half_s_curv_cal) {
            this->curv_cal.start_curv_index = i;
            break;
        }
    }
    for (count_type i = (this->x_size - 1); i >= 0; --i) {
        if ((this->s[this->x_size - 1] - this->s[i]) >= this->params.half_s_curv_cal) {
            this->curv_cal.end_curv_index = i;
            break;
        }
    }
    if ((this->curv_cal.start_curv_index == 0) || (this->curv_cal.end_curv_index == this->x_size) || (this->curv_cal.start_curv_index > this->curv_cal.end_curv_index)) {
        this->warn.update.items.curv_index_warn = true;
        return;
    }

    this->curv_cal.curv_one_x.assign(this->x_size, 0);
    this->curv_cal.curv_one_y.assign(this->x_size, 0);
    this->curv_cal.curv_thr_x.assign(this->x_size, 0);
    this->curv_cal.curv_thr_y.assign(this->x_size, 0);

    for (count_type i = this->curv_cal.start_curv_index; i <= this->curv_cal.end_curv_index; ++i) {
        for (count_type j = i; j >= 0; --j) {
            if ((this->s[i] - this->s[j]) >= this->params.half_s_curv_cal) {
                this->curv_cal.ratio = (this->s[i] - this->s[j] - this->params.half_s_curv_cal) / this->interval[j + 1];
                this->curv_cal.curv_one_x[i] = x_points[j] + this->curv_cal.ratio * (x_points[j + 1] - x_points[j]);
                this->curv_cal.curv_one_y[i] = y_points[j] + this->curv_cal.ratio * (y_points[j + 1] - y_points[j]);
                break;
            }
        }
        for (count_type k = i; k < this->x_size; ++k) {
            if ((this->s[k] - this->s[i]) >= this->params.half_s_curv_cal) {
                this->curv_cal.ratio = (this->params.half_s_curv_cal - (this->s[k - 1] - this->s[i])) / this->interval[k];
                this->curv_cal.curv_thr_x[i] = x_points[k - 1] + this->curv_cal.ratio * (x_points[k] - x_points[k - 1]);
                this->curv_cal.curv_thr_y[i] = y_points[k - 1] + this->curv_cal.ratio * (y_points[k] - y_points[k - 1]);
                break;
            }
        }
    }
}

cal_type Points_Attributes::get_point_curv(const count_type i, const std::vector<cal_type> &x_points, const std::vector<cal_type> &y_points) {
    double_t a, b, c;
    double_t s;
    double_t k;
    cal_type curv;

    a = this->interval[i];
    b = this->interval[i + 1];
    c = sqrt(pow((x_points[i + 1] - x_points[i - 1]), 2.0) + pow((y_points[i + 1] - y_points[i - 1]), 2.0));

    s = (a + b + c) / 2.0;
    k = sqrt(fabs(s * (s - a) * (s - b) * (s - c)));
    curv = 4 * k / (a * b * c);
    return curv;
}

void Points_Attributes::curv_reproduce() {
    for (count_type i = 0; i < this->x_size; ++i) {
        if (!this->curv_cal.curv_zero_sign[i]) {
            cal_type tmp1;
            tmp1 = 0.0;
            for (count_type j = i; j < this->x_size; ++j) {
                if (this->curv_cal.curv_zero_sign[j]) {
                    break;
                }
                i = j;
                if (this->curv_cal.curv_limited[j] > tmp1) {
                    tmp1 = this->curv_cal.curv_limited[j];
                }
            }
            for (count_type k = i; k >= 0; --k) {
                if (this->curv_cal.curv_zero_sign[k]) {
                    break;
                }
                this->curv_cal.curv_limited[k] = tmp1;
            }
        }
    }
}

}