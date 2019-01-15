#ifndef CONSTRAINED_SPEED_CSMETHOD2_HPP
#define CONSTRAINED_SPEED_CSMETHOD2_HPP

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "constrained_speed/cs_method2Parameters.h"
#include "plan2control_msgs/Trajectory.h"
#include "lanelet_map_msgs/Way.h"
#include "control_msgs/GetECUReport.h"
#include "control_msgs/SteerCmd.h"

#include "SLog.hpp"
#include "SJerk.hpp"
#include "SPoints.hpp"
#include "SThreePointsCurvature.hpp"
#include "STime.hpp"
#include "Toyota.hpp"

namespace constrained_speed {

#define LOG_INFO LOG(INFO)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define LOG_WARN LOG(WARNING)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define LOG_ERROR LOG(ERROR)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define LOG_FATAL LOG(FATAL)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define FIXED std::setiosflags(std::ios::fixed)<<

#define PROCESS_PERIOD 0.1
#define ISSUE_MODE "direct" // direct cycle
#define TIME_CHECK_PERIOD 0.2

enum class direction {
    stop = 0,
    forward = 1,
    backward = 2
};

struct params {
    double_t acc_lat;
    double_t acc_max;
    double_t acc_min;
    double_t safe_dec;
    double_t v_max;
    std::string road_net_topic;
};

class CSMethod2 {
public:
    CSMethod2(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle);

private:
    params yaml_params_;

    direction direction_;
    std::vector<double_t> v_;
    double_t cur_speed_;
    double_t cur_acc_;

    size_t points_size_;
    std::vector<double_t> x_points_;
    std::vector<double_t> y_points_;
    std::vector<std::vector<double_t>> intervals_arc_lengths_;
    std::vector<double_t> &intervals_ = intervals_arc_lengths_[0];
    std::vector<double_t> &arc_lengths_ = intervals_arc_lengths_[1];

    std::vector<double_t> curvatures_;

    shawn::SLog sLog_;

    void glogInit();
    void paramsInit();
    void planning();

    bool pointsPretreat();
    bool pointsAttributes();
    bool setVMax();

    bool limitLatAcc();
    bool smoothSpeed();

    bool blindHandle();
    bool durex();

    void pathCb(const plan2control_msgs::Trajectory msg);
    void roadnetCb(const lanelet_map_msgs::Way msg);
    void ecuCb(const control_msgs::GetECUReport msg);
    void steerCmdCb(const control_msgs::SteerCmd msg);

    ros::Subscriber path_sub_;
    ros::Subscriber ecu_sub_;
    ros::Subscriber roadnet_sub_;
    ros::Subscriber steer_cmd_sub_;

    ros::Publisher traj_pub_;
    ros::Publisher ctrl_pub_;

    ros::Timer process_timer_;
    ros::Timer check_timer_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    plan2control_msgs::Trajectory trajectory_;
    lanelet_map_msgs::Way way_;
    control_msgs::GetECUReport getECUReport_;
    control_msgs::SteerCmd steerCmd_;


    void time_check();
    void setHandle();

    shawn::STime sub_times_;
    shawn::handle path_sub_handle_;
    shawn::handle ecu_sub_handle_;
    shawn::handle roadnet_sub_handle_;
    shawn::handle steer_cmd_sub_handle_;


    cs_method2Parameters params_;
    /// dynamic reconfigure service
    dynamic_reconfigure::Server<cs_method2Config> reconfigSrv_;
    void reconfigureRequest(cs_method2Config & config, uint32_t level);


    bool no_msg_;
    bool limitBySteer();
    bool calTime();
    bool calAcc();
    bool calJerk();
    std::vector<double_t> time_;
    std::vector<double_t> time_interval_;
    std::vector<double_t> acc_;
    std::vector<double_t> jerk_;
    bool publish();
};

}

#endif //CONSTRAINED_SPEED_CSMETHOD2_HPP