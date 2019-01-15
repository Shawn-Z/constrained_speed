#ifndef CONSTRAINED_SPEED_TOYOTA_HPP
#define CONSTRAINED_SPEED_TOYOTA_HPP

#include <ros/ros.h>
#include <speed_ctrl_msgs/speed_ctrl.h>

namespace constrained_speed {

struct toyota_issue {
    double_t v;
    double_t acc;
};

class Toyota {
public:
    void publish(ros::NodeHandle nh, std::vector<double_t> &p_time, std::vector<double_t> &p_v, std::vector<double_t> &p_acc, double_t p_acc_delay, double_t p_dec_delay, bool forward) {
        static ros::Publisher publisher = nh.advertise<speed_ctrl_msgs::speed_ctrl>("/speed_plan", 1);
        static speed_ctrl_msgs::speed_ctrl speed_ctrl;
        toyota_issue issue = issueCal(p_time, p_v, p_acc, p_acc_delay, p_dec_delay);
        brakeAmount(p_v, issue);
        speed_ctrl.direction = forward;
        speed_ctrl.cur_speed = p_v[0];
        speed_ctrl.cur_acc = p_acc[0];
        speed_ctrl.issue_v = issue.v;
        speed_ctrl.issue_acc = issue.acc;
        publisher.publish(speed_ctrl);
    }

private:
    toyota_issue issueCal(std::vector<double_t> &p_time, std::vector<double_t> &p_v, std::vector<double_t> &p_acc, double_t p_acc_delay, double_t p_dec_delay) {
        size_t size = p_v.size();
        size_t delay_index = size - 2;
        double_t delay_ratio = 1.0;
        for (size_t i = 1; i < size; ++i) {
            if (p_time[i] > p_dec_delay) {
                delay_index = i - 1;
                delay_ratio = (p_dec_delay - p_time[i - 1]) / (p_time[i] - p_time[i - 1]);
                break;
            }
        }
        toyota_issue issue;
        issue.v = p_v[delay_index] + delay_ratio * (p_v[delay_index + 1] - p_v[delay_index]);
        issue.acc = p_acc[delay_index] + delay_ratio * (p_acc[delay_index + 1] - p_acc[delay_index]);
        for (size_t j = 1; j <= delay_index; ++j) {
            issue.acc = std::min(issue.acc, p_acc[j]);
        }
        if (issue.v < p_v[0]) {
            return issue;
        }
        for (size_t i = 1; i < size; ++i) {
            if (p_time[i] > p_acc_delay) {
                delay_index = i - 1;
                delay_ratio = (p_acc_delay - p_time[i - 1]) / (p_time[i] - p_time[i - 1]);
                break;
            }
        }
        issue.v = p_v[delay_index] + delay_ratio * (p_v[delay_index + 1] - p_v[delay_index]);
        issue.acc = p_acc[delay_index] + delay_ratio * (p_acc[delay_index + 1] - p_acc[delay_index]);
        return issue;
    }

    void brakeAmount(std::vector<double_t> &p_v, toyota_issue &p_issue) {
        double_t brake_distinguish = -0.2;
        double_t acc_time_limit = 0.5;
        double_t revise_acc = -0.2;
        double_t brake_amount_amplify = 1.0;

        static double_t brake_amount = 0.0;
        static std::vector<double_t> acc_times;
        static std::vector<double_t> dec_times;
        static std::vector<double_t> decs;
        static double_t dec_start_v = 0.0;
        if ((fabs(p_v[0]) < 0.1) || (fabs(p_issue.v) < 0.1)) {
            brake_amount = 0.0;
            acc_times.clear();
            dec_times.clear();
            decs.clear();
            return;
        }
        if (p_issue.acc >= brake_distinguish) {
            dec_times.clear();
            decs.clear();
            if (acc_times.empty()) {
                acc_times.assign(2, ros::Time::now().toSec());
            } else {
                acc_times[1] = ros::Time::now().toSec();
            }
            double_t acc_time = acc_time = acc_times[1] - acc_times[0];
            if (acc_time >= acc_time_limit) {
                brake_amount = 0.0;
                dec_times.clear();
                decs.clear();
            }
        } else {
            acc_times.clear();
            if (dec_times.empty()) {
                dec_times.assign(2, ros::Time::now().toSec());
                decs.assign(2, p_issue.acc);
                dec_start_v = p_issue.acc;
            } else {
                dec_times.erase(dec_times.begin());
                decs.erase(decs.begin());
                dec_times.emplace_back(ros::Time::now().toSec());
                decs.emplace_back(p_issue.acc);
            }
            if (dec_start_v - brake_amount < p_issue.v) {
                p_issue.acc = revise_acc;
            } else {
                brake_amount += brake_amount_amplify * 0.5 * fabs(decs[0] + decs[1]) * (dec_times[1] - dec_times[0]);
            }
        }
    }
};

}

#endif //CONSTRAINED_SPEED_TOYOTA_HPP