#ifndef CONSTRAINED_SPEED_THREEONEPUBLISH_HPP
#define CONSTRAINED_SPEED_THREEONEPUBLISH_HPP

#include <ros/ros.h>
#include "three_one_msgs/control_speed.h"

namespace constrained_speed {

struct three_one_issue {
    double_t v;
    double_t acc;
};

class ThreeOnePublish {
    three_one_issue publish(ros::NodeHandle nh, std::vector<double_t> &p_time, std::vector<double_t> &p_v, std::vector<double_t> &p_acc, double_t p_delay, bool forward) {
        static ros::Publisher publisher = nh.advertise<three_one_msgs::control_speed>("/speed_plan", 1);
        static three_one_msgs::control_speed control_speed;
        three_one_issue issue = issueCal(p_time, p_v, p_acc, p_delay);
        if (issue.v < 0.0001) {
            return issue;
        }
        issue.v = std::max(issue.v, 0.1);
        control_speed.priority = 0;
        control_speed.gear = forward? 1: 2;
        control_speed.speed = issue.v;
        publisher.publish(control_speed);
        return issue;
    }

    three_one_issue issueCal(std::vector<double_t> &p_time, std::vector<double_t> &p_v, std::vector<double_t> &p_acc, double_t p_delay) {
        size_t size = p_v.size();
        size_t delay_index = size - 2;
        double_t delay_ratio = 1.0;
        for (size_t i = 1; i < size; ++i) {
            if (p_time[i] > p_delay) {
                delay_index = i - 1;
                delay_ratio = (p_delay - p_time[i - 1]) / (p_time[i] - p_time[i - 1]);
                break;
            }
        }
        three_one_issue issue;
        issue.v = p_v[delay_index] + delay_ratio * (p_v[delay_index + 1] - p_v[delay_index]);
        issue.acc = p_acc[delay_index] + delay_ratio * (p_acc[delay_index + 1] - p_acc[delay_index]);
        return issue;
    }
};

}

#endif //CONSTRAINED_SPEED_THREEONEPUBLISH_HPP