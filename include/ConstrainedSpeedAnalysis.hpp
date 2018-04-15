#ifndef CONSTRAINED_SPEED_CONSTRAINEDSPEEDANALYSIS_HPP
#define CONSTRAINED_SPEED_CONSTRAINEDSPEEDANALYSIS_HPP

#include <nodelet/nodelet.h>
#include <rosbag/bag.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <ctime>
#include "speed_debug_msgs/speed_debug.h"

namespace constrained_speed_analysis {
    class ConstrainedSpeedAnalysis : public nodelet::Nodelet {
    private:
        char *home_;
        char *bag_path_;
        time_t time_now_;
        char *c_time_now_;
        const char *cc_time_now_;
        std::string s_time_now_;

        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        rosbag::Bag bag_;

        virtual void onInit();

        void callback(const speed_debug_msgs::speed_debug msg);
    };
}
#endif //CONSTRAINED_SPEED_CONSTRAINEDSPEEDANALYSIS_HPP
