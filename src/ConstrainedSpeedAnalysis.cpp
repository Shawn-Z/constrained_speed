#include "ConstrainedSpeedAnalysis.hpp"
#include <sys/types.h>
#include <sys/stat.h>

namespace constrained_speed_analysis {
    void ConstrainedSpeedAnalysis::onInit() {
        this->home_ = getenv("HOME");
        this->time_now_ = time(NULL);
        this->c_time_now_ = ctime(&time_now_);
        this->s_time_now_ = c_time_now_;
        this->s_time_now_ = this->s_time_now_.substr(0, this->s_time_now_.length() - 1);
        this->cc_time_now_ = s_time_now_.c_str();
        this->c_time_now_ = const_cast<char *>(this->cc_time_now_);
        this->bag_path_ = strcat(this->home_, "/catkin_ws/src/constrained_speed/src");
        chdir(this->bag_path_);
        if (bool(access("bagfiles", 0))) {
            mkdir("bagfiles", 0777);
        }
        this->bag_path_ = strcat(this->bag_path_, "/bagfiles");
        chdir(this->bag_path_);
        this->bag_path_ = getcwd(NULL, 0);
        NODELET_INFO_STREAM("The rosbag file will be locate at " << this->bag_path_);
        NODELET_INFO_STREAM("The bagfile name will be " << this->c_time_now_ << ".bag");
        this->nh_ = getNodeHandle();
        this->bag_.open(strcat(this->c_time_now_, ".bag"), rosbag::bagmode::Write);
        this->sub_ = this->nh_.subscribe("/speed_debug", 10, &ConstrainedSpeedAnalysis::callback, this);
    }

    void ConstrainedSpeedAnalysis::callback(const speed_debug_msgs::speed_debug msg) {
        this->bag_.write("speed_debug", ros::Time::now(), msg);
    }
}
PLUGINLIB_DECLARE_CLASS(constrained_speed, ConstrainedSpeedAnalysis,
                        constrained_speed_analysis::ConstrainedSpeedAnalysis,
                        nodelet::Nodelet)
