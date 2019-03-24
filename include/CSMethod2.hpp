#ifndef CONSTRAINED_SPEED_CSMETHOD2_HPP
#define CONSTRAINED_SPEED_CSMETHOD2_HPP

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "constrained_speed/cs_method2Parameters.h"
#include "plan2control_msgs/Trajectory.h"
#include "lanelet_map_msgs/Way.h"

#include "speed_debug_msgs/speed_debug.h"
#include "three_one_msgs/report.h"
#include "three_one_msgs/control.h"

#include "SLog.hpp"
#include "SJerk.hpp"
#include "SPoints.hpp"
#include "SThreePointsCurvature.hpp"
#include "STime.hpp"
//#include "Toyota.hpp"
#include "ThreeOnePublish.hpp"

#include "DEFINEs.hpp"

namespace constrained_speed {

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
    /// \brief  store params from file(yaml)
    params yaml_params_;

    /// \brief  store the direction from the path
    direction direction_;
    /// \brief  the speed profile
    std::vector<double_t> v_;
    /// \brief  current speed of the car
    double_t cur_speed_;
    /// \brief  current acceleration of the car, signed
    double_t cur_acc_;

    /// \brief  store the points number of the received path
    size_t points_size_;
    /// \brief  store the x coordinates of the received path
    std::vector<double_t> x_points_;
    /// \brief  store the y coordinates of the received path
    std::vector<double_t> y_points_;
    /// \brief  store the intervals and arc-lengths of the received path
    std::vector<std::vector<double_t>> intervals_arc_lengths_;
    /// \brief  store the intervals of the received path
    std::vector<double_t> intervals_;
    /// \brief  store the arc-lengths of the received path
    std::vector<double_t> arc_lengths_;

    /// \brief  store the curvatures of the received path
    std::vector<double_t> curvatures_;

    /// \brief  log tool
    shawn::SLog sLog_;

    /// \brief  init the log tool
    void glogInit();
    /// \brief  init the params
    void paramsInit();
    /// \brief  the function of speed planning
    void planning();

    /// \brief  pretreat the received path
    bool pointsPretreat();
    /// \brief  calculate some attributes of the received path
    bool pointsAttributes();
    /// \brief  set v max of the trajectory
    bool setVMax();

    /// \brief  limit the latitude acceleration of the trajectory
    bool limitLatAcc();
    /// \brief  smooth the speed profile along the path
    bool smoothSpeed();

    /// \brief  this function is not from theory, but should be used in practice
    bool blindHandle();
    /// \brief  the car should have ability to avoid from danger
    bool durex();

    /// \brief  callback function of the path
    void pathCb(const plan2control_msgs::Trajectory msg);
    /// \brief  callback function of the roadnet
    void roadnetCb(const lanelet_map_msgs::Way msg);
    /// \brief  callback function of the steer command
    void steerCmdCb(const three_one_msgs::control_steer msg);

    /// \brief  callback function of ecu on cw platform
    void three_one_ecuCb(const three_one_msgs::report msg);

    /// \brief  the subscriber of the path
    ros::Subscriber path_sub_;
    /// \brief  the subscriber of the ecu
    ros::Subscriber ecu_sub_;
    /// \brief  the subscriber of the roadnet
    ros::Subscriber roadnet_sub_;
    /// \brief  the subscriber of the steer command
    ros::Subscriber steer_cmd_sub_;
    /// \brief  the subscriber of the ecu on cw platform
    ros::Subscriber three_one_ecu_sub_;

    /// \brief  the publisher of the trajectory
    ros::Publisher traj_pub_;

    /// \brief  the process timer of speed planning
    ros::Timer process_timer_;
    /// \brief  the timer of time check
    ros::Timer check_timer_;

    /// \brief ros nodehandle
    ros::NodeHandle nh_;
    /// \brief ros private nodehandle
    ros::NodeHandle private_nh_;

    /// \brief  ros msg of trajectory
    plan2control_msgs::Trajectory trajectory_;
    /// \brief ros msg of roadnet
    lanelet_map_msgs::Way way_;
    /// \brief ros msg of steer command
    three_one_msgs::control_steer control_steer_;


    /// \brief  check the time period of variety functions
    void time_check();
    /// \brief  set handles
    void setHandle();

    /// \brief time check tool for subscribing
    shawn::STime sub_times_;
    /// \brief path subscriber handle
    shawn::handle path_sub_handle_;
    /// \brief ecu subscriber handle
    shawn::handle ecu_sub_handle_;
    /// \brief roadnet subscriber handle
    shawn::handle roadnet_sub_handle_;
    /// \brief steer command subscriber handle
    shawn::handle steer_cmd_sub_handle_;


    /// \brief dynamic reconfigure params
    cs_method2Parameters params_;
    /// \brief  dynamic reconfigure service
    dynamic_reconfigure::Server<cs_method2Config> reconfigSrv_;
    /// \brief callback function of dynamic reconfigure
    void reconfigureRequest(cs_method2Config & config, uint32_t level);


    /// \brief mark if no msg received from ros
    bool no_msg_;
    /// \brief limit speed profile by steer command
    bool limitBySteer();
    /// \brief calculate time of neighboring points
    bool calTime();
    /// \brief calculate acceleration of speed profile
    bool calAcc();
    /// \brief calculate jerk of speed profile
    bool calJerk();
    /// \brief store time attribute of speed profile
    std::vector<double_t> time_;
    /// \brief store time interval attribute of speed profile
    std::vector<double_t> time_interval_;
    /// \brief store acceleration attribute of speed profile
    std::vector<double_t> acc_;
    /// \brief store jerk attribute of speed profile
    std::vector<double_t> jerk_;
    /// \brief publish results to ros
    bool publish();
    /// \brief additional publish
    void additionPublish(std::vector<double_t> issue);
};

}

#endif //CONSTRAINED_SPEED_CSMETHOD2_HPP