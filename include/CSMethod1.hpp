//#define TOYOTA
//#define TANK_6T

#ifndef CONSTRAINED_SPEED_CSMETHOD1_HPP
#define CONSTRAINED_SPEED_CSMETHOD1_HPP

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "constrained_speed/cs_method1Parameters.h"
#include "plan2control_msgs/Trajectory.h"
#include "speed_debug_msgs/speed_debug.h"
#include "dymidetect2speed_msgs/dymicol_point.h"
#include "sensor_driver_msgs/GpswithHeading.h"
#include "lanelet_map_msgs/Way.h"
#include "Points_Attributes.hpp"
#include "speed_ctrl_msgs/speed_ctrl.h"
#ifdef TOYOTA
#include "control_msgs/GetECUReport.h"
#endif
#ifdef TANK_6T
#include "control_msgs/GetECUReport.h"
#endif
#ifdef NORTH_COLA
#include "control_msgs/GetECUReport.h"
#endif

namespace constrained_speed {

typedef double_t vel_type;
typedef int16_t count_type;

enum class gear_type {P, N, R, D1, D2, D3, D4, D5, unknown};

enum class running_mode_type {
    default_mode = 0,
    general_ahead = 1,
    general_retreat = 2,
    dynamic_collision_ahead = 3,
    dynamic_collision_retreat = 4,
    foggy_ahead = 5,
    foggy_retreat = 6,
    inspect_ahead = 7,
    inspect_retreat = 8,
    search_ahead = 9,
    search_retreat = 10,
    search_direction_diff = 11,
    unknown
};

struct msg_update_type {
    union {
        struct {
            bool path: 1;
            bool speed: 1;
        } items;
        uint8_t result;
    } update;
    uint8_t yes;
    struct {
        bool collision;
        bool ramp;
    } addition;
};

struct error_mark_type {
    union {
        struct {
            bool same_direction_points_num: 1;
            bool pointsAttributes_calculate: 1;
            bool reach_safe_distance: 1;
            bool direction_change: 1;
            bool desired_direction_wrong: 1;
        } items;
        uint8_t result;
    } update;
};

struct fake_type {
    bool p_fake;
    bool p_speed_fake;
    bool p_acc_fake;
    bool p_steer_fake;
    bool p_issue_acc_fake;
    bool p_issue_direction_fake;
    vel_type p_fake_speed;
    vel_type p_fake_acc;
    vel_type p_fake_steer;
    vel_type p_fake_issue_acc;
    gear_type p_fake_issue_direction;
};

struct cur_speed_type {
    vel_type speed;
    std::vector<vel_type> seq;
};

struct cur_acc_type {
    vel_type acc;
};

struct cur_steer_type {
    double_t steer;
    double_t fabs_max;
    std::vector<double_t> seq;
};

struct durex_type {
    count_type safe_redundancy_index;
    vel_type p_safe_redundancy;
    vel_type at;
    vel_type effective_s;
    vel_type p_safe_dec;
    vel_type p_dec_delay_time;
    vel_type v_limit;
};

struct set_v_max_type {
    std::vector<vel_type> v_max_dynamic;
    vel_type p_v_max_general;
    vel_type p_jerk_O_S_vex_max;
    vel_type p_slide_dec;
    vel_type p_jerk_O_S_cave_max;
    vel_type p_v_max_cycle_diff;
    vel_type p_v_max_cycle_times;
};

struct steer_type {
    vel_type v_limit;
    vel_type p_steer_lat_acc;
    vel_type p_wheelbase;
    vel_type p_steer_amplify;
};

struct ramp_type {
//    int8_t update_count;
    std::vector<double_t> slopes;
    double_t pitch;
    double_t roll;
//    double_t slope;
//    double_t slope_d;
//    double_t roll_ratio;
//    vel_type v_limit_pitch;
//    vel_type v_limit_roll;
    double_t start_slope;
    vel_type v_limit;
};

struct lat_acc_type {
    vel_type p_acc_lat_max;
    std::vector<vel_type> v_limit;
};

struct blind_type {
    count_type end_index;
    vel_type area;
    int p_blind_mode;
//    bool p_blind_reactivate;
    vel_type p_blind_time;
    vel_type p_blind_slide_dec;
    std::vector<vel_type> v_blind;
};

struct dynamic_collision_type {
    bool collision;
    int8_t update_count;
    vel_type distance;
    vel_type real_distance;
    std::vector<vel_type> v_limit;
    vel_type p_safe_redundancy;
    vel_type p_dec_delay;
};

struct lon_acc_type {
    std::vector<vel_type> v_limit;
    vel_type p_jerk_N_A_cave;
    vel_type p_acc_lon_max;
};

struct slide_dec_type {
    std::vector<vel_type> v_limit;
    vel_type p_jerk_N_S_cave;
    vel_type p_slide_dec;
    vel_type c_regular_min_speed;
    vel_type p_v_init_diff;
    int p_remove_dec_mode;
    bool slide_gen_warn;
};

struct lon_dec_type {
    std::vector<vel_type> v_limit;
    vel_type p_jerk_N_D_cave;
    vel_type p_dec_lon_max;
    int p_remove_dec_mode;
    vel_type p_slide_dec;
    vel_type c_regular_min_speed;
    vel_type p_v_init_diff;
    bool dec_gen_warn;
};

struct dec_fail_type {
    std::vector<vel_type> v_limit;
    vel_type p_dec_lon_max;
    int p_remove_dec_mode;
    vel_type c_regular_min_speed;
    bool dec_fail_gen_warn;
    bool dec_fail_gen_warn2;
    vel_type p_slide_dec;
    vel_type p_v_init_diff;
    vel_type p_safe_dec;
};

struct jerk_type {
    std::vector<vel_type> v_limit;
    vel_type p_jerk_N_A_vex;
    vel_type p_jerk_N_D_vex;
    vel_type p_jerk_N_S_vex;
    vel_type p_jerk_cycle_diff;
    vel_type p_jerk_cycle_times;
};

struct time_type {
    std::vector<vel_type> time;
    std::vector<vel_type> interval;
};

struct issue_type {
    vel_type acc;
    vel_type v;
    gear_type direction;
    count_type delay_index;
    double_t ratio;
    vel_type p_acc_delay;
    vel_type p_dec_delay;
    vel_type p_min_brake_dec;
    vel_type p_acc_lon_max;
    vel_type p_safe_dec;
    bool p_not_pub_dead_zone;
};

struct brake_amount_type {
    int p_brake_amount_mode;
    double_t amount;
    vel_type dec_start_v;
    vel_type p_brake_distinguish;
    std::vector<double_t> acc_times;
    std::vector<double_t> dec_times;
    std::vector<vel_type> decs;
    double_t acc_time;
    double_t p_acc_time_limit;
    double_t p_brake_amount_amplify;
    bool p_brake_amplify_revise;
    double_t p_brake_amount_multiple_coefficient;
    vel_type p_slide_dec;
};

struct on_off_type {
    int p_issue_mode;
    bool p_verbose;
    bool p_program_halt;
//        bool p_curve_reproduce;
//        bool p_curv_modify;
//        bool p_curv_increase;
    bool p_enable_debug;
};

struct time_check_type {
    double_t rc_ros_time;
    double_t pub_ros_time;
    double_t rc_way_time;
};

struct assist_params_type {
    double_t smart_safe_distance;
    double_t total_arc_length;
};

struct segmentRoad_type {
    vel_type v_limit;
    std::vector<vel_type> x_points;
    std::vector<vel_type> y_points;

    vel_type result_curvature_max;

    count_type consider_points_num;
};

class CSMethod1 {
public:
    CSMethod1(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle);

private:
    void decide_running_mode();
    std::map<running_mode_type, std::string> running_mode_strings_;

    void find_first_min_v();

    ros::Timer check_timer_;
    ros::Timer run_timer_;
    void time_check();

    struct {
        segmentRoad_type segmentRoad;

        bool warn_diff_direction;


        running_mode_type running_mode;
//        vehicle_params_type vehicle_params;
        assist_params_type assist_params;

        uint32_t cycle_count;

        cur_speed_type cur_speed;
        cur_acc_type cur_acc;
        cur_steer_type cur_steer;

        msg_update_type msg_udt;
        error_mark_type error_mark;

        on_off_type on_off;

        fake_type fake;

        std::vector<vel_type> acc;

        std::vector<vel_type> x_points;
        std::vector<vel_type> y_points;

        count_type point_num;
        gear_type direction;
        gear_type cur_direction;
        std::vector<vel_type> s;
        std::vector<vel_type> interval;
        std::vector<vel_type> curv_final;
        std::vector<bool> curv_zero_sign;

        std::vector<vel_type> v_constrained;
        durex_type durex;
        set_v_max_type set_v_max;
        steer_type steer;
        ramp_type ramp;
        lat_acc_type lat_acc;
        blind_type blind;
        dynamic_collision_type dynamic_collision;
        lon_acc_type lon_acc;
        slide_dec_type slide_dec;
        lon_dec_type lon_dec;
        dec_fail_type dec_fail;
        jerk_type jerk;
        time_type time;
        issue_type issue;
        brake_amount_type brake_amount;
        time_check_type time_check;

        std::vector<vel_type> tmp_vel_data1;
        std::vector<vel_type> tmp_vel_data2;
        std::vector<vel_type> tmp_acc;
        std::vector<vel_type> tmp_v;

    } super_points_;

    /// \brief  string of each gear, to display
    std::map<gear_type, std::string> gear_strings_;

    ////  points attributes
    /// \brief  params of points attributes
    points_attributes::params_type point_attributes_params_;
    /// \brief  object of class points attributes
    points_attributes::Points_Attributes pointsAttributes_;

    //// rosparam handler
    /// used to receive parameters for .yaml and dynamic reconfigure
    cs_method1Parameters params_;
    /// dynamic reconfigure service
    dynamic_reconfigure::Server<cs_method1Config> reconfigSrv_;

    //// ROS
    /// \brief ROS node handle
    ros::NodeHandle nh_;
    /// \brief ROS private node handle
    ros::NodeHandle private_nh_;
    /// \brief Publisher, trajectory
    ros::Publisher traj_pub_;
    ros::Publisher ctrl_pub_;
    /// \brief Publisher, to analysis package
    ros::Publisher debug_pub_;
    /// \brief Subscriber
    ros::Subscriber path_sub_;
    /// \brief Subscriber
    ros::Subscriber speed_sub_;
    /// \brief Subscriber
    ros::Subscriber collision_sub_;
    /// \brief Subscriber
    ros::Subscriber ramp_sub_;
    /// \brief Subscriber;
    ros::Subscriber segmentRoad_sub_;


    //// msgs
    /// \brief msg
    plan2control_msgs::Trajectory trajectory_;
    /// \brief msg
    speed_ctrl_msgs::speed_ctrl speed_ctrl_;
    dymidetect2speed_msgs::dymicol_point dymicol_point_;
    sensor_driver_msgs::GpswithHeading gpswithHeading_;
    /// \brief msg
    speed_debug_msgs::speed_debug speed_debug_;
    /// \brief msg
    speed_debug_msgs::super_point speed_debug_point_;

    lanelet_map_msgs::Way way_;


#ifdef TOYOTA
    control_msgs::GetECUReport getECUReport_;
#endif
#ifdef NORTH_COLA
    control_msgs::GetECUReport getECUReport_;
#endif
#ifdef TANK_6T
    control_msgs::GetECUReport getECUReport_;
#endif


    //// Functions
    void cal_assist_params();

    void limit_segmentRoad();
    ////  points attributes
    /// \brief  params of points attributes
    points_attributes::params_type point_attributes_params_segmentRoad_;
    /// \brief  object of class points attributes
    points_attributes::Points_Attributes pointsAttributes_segmentRoad_;


    /// \brief callback of dynamic reconfigure
    /// \param [in] config  the configure from rqt-dynamic-reconfigure
    void reconfigureRequest(cs_method1Config&, uint32_t);
    /// \brief  init something in constructor
    void init();
    /// \brief  callback function of path/trajectory subscriber
    /// \param [in] msg the trajectory message
    void pathSubCb(const plan2control_msgs::Trajectory msg);


    /// \brief  callback function of segment Road
    void segmentRoadCb(const lanelet_map_msgs::Way msg);
    /// \brief  callback function of dynamic collision
    void collisionCb(const dymidetect2speed_msgs::dymicol_point msg);
    /// \brief  callback function of Ramp subscriber
    void rampCb(const sensor_driver_msgs::GpswithHeading msg);
    /// \brief  entrance of speed planning
    /// \details  there is two way to invoke this function, one the after the path received, the other is use cycle.
    void timerCb();
    /// \brief  init something in planning
    /// \details  mainly init the params form dynamic reconfigure
//    void assist_params_init();
    /// \brief  reset the state of variables
    void state_reset_before();
    /// \brief  reset the state of variables
    void state_reset_after();
    /// \brief  calculate variable attributes of received path
    void points_attributes();

    /// \brief  sub entrance of speed planning
    void general_planning();

    /// \brief  calculate safe speed
    void durex();
    /// \brief  set the max velocity
    void set_v_max();
    /// \brief  apply the steer constraint
    void limit_by_steer();
    void limit_by_ramp();
    /// \brief  apply the latitude acceleration constraint
    void limit_lat_acc();
    /// \brief  adjust to the enormous brake delay
    void blind_handle();
    /// \brief dynamic collision
    void dynamic_collision();
    /// \brief  apply the longitude acceleration constraint
    void limit_lon_acc();
    /// \brief  adjust to the brake dead zone
    /// \details  first try to generate speed curve without brake because of brake dead zone
    void limit_slide_dec();
    /// \brief  apply the longitude deceleration constraint
    void limit_lon_dec();
    /// \brief  handle the situation if can not generate speed curve using longitude deceleration
    void dec_fail_handle();
    /// \brief  apply the jerk constraints
    /// \details  actually, some jerk constraints have been applied in acceleration constraints, here apply the rest
    void limit_jerk();
    /// \brief  calculate the acceleration of the generated speed curve
    void cal_acc();
    /// \brief  calculate the time of the generated speed curve
    void cal_time();
    /// \brief  calculate the acceleration to be issued
    void issue_cal();
    /// \brief  predict when to release braking by amount brake
    void brake_amount();
    /// \brief  publish the trajectory
    void publish();
    /// \brief  deal with the module -- analysis speed
    void debug_handle();
    /// \brief  fake some current state
    void fake_state();
    /// \brief  fake the quantity of issue
    void fake_issue();

#ifdef TOYOTA
    /// \brief  callback function of speed/steer/ecu subscriber
    /// \param [in] msg the trajectory message
    void speedCb(const control_msgs::GetECUReport msg);
#endif
#ifdef NORTH_COLA
    /// \brief  callback function of speed/steer/ecu subscriber
    /// \param [in] msg the trajectory message
    void speedCb(const control_msgs::GetECUReport msg);
#endif
#ifdef TANK_6T
    /// \brief  callback function of speed/steer/ecu subscriber
    /// \param [in] msg the trajectory message
    void speedCb(const control_msgs::GetECUReport msg);
#endif

    //// Assist Functions
    /// \brief  get velocity from state of former point
    vel_type jerk_speed_solution(const vel_type v0, const vel_type a0, const vel_type s, const vel_type jerk, const vel_type v_up, const vel_type v_down, const vel_type a_up, const vel_type a_down);
    /// \brief  get velocity of middle point to fit jerk constraint
    vel_type jerk_speed_solution(const vel_type v_1, const vel_type v1, const vel_type s0, const vel_type s1, const vel_type jerk);

};

}

#endif //CONSTRAINED_SPEED_CSMETHOD1_HPP
