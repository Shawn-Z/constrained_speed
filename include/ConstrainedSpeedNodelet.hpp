#ifndef CONSTRAINED_SPEED_NODELET_CONSTRAINEDSPEEDNODELET_HPP
#define CONSTRAINED_SPEED_NODELET_CONSTRAINEDSPEEDNODELET_HPP

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <vector>
#include <pluginlib/class_list_macros.h>
#include "math.h"
#include "nav_msgs/Path.h"
#include "plan2control_msgs/Trajectory.h"
#include "speed_debug_msgs/speed_debug.h"
#include "sensor_msgs/Imu.h"
#include "control_msgs/GetECUReport.h"
#include "sensor_driver_msgs/GpswithHeading.h"
#include "time.hpp"

namespace constrained_speed {
    class ConstrainedSpeedNodelet : public nodelet::Nodelet {
    public:

    private:

        // update markers
        bool is_path_update_;
        bool is_speed_update_;

        // variables for trajectory topic
        plan2control_msgs::Trajectory trajectory_;
        plan2control_msgs::Traj_Node traj_node_;
        speed_debug_msgs::speed_debug speed_debug_;

        // variables for trajectory
        std::vector<double> point_x_, point_y_;
        std::vector<double> curvature_;
        std::vector<double> s_;
        std::vector<bool> curvature_sign_;
        unsigned int points_num_;

        // variables for velocity profile
        std::vector<double> v_max_;
        std::vector<double> v_limit_lat_acc_;
        std::vector<double> v_limit_lon_acc_;
        std::vector<double> v_limit_lon_adc_;
        std::vector<double> v_limit_lon_jerk_;

        std::vector<double> vf_;
        std::vector<double> v0_;

        // ros parameters
        ros::NodeHandle nh_, private_nh_;
        ros::Subscriber path_sub_, speed_sub_, acc_sub_, gps_sub_;
        ros::Publisher traj_pub_, debug_pub_;
        ros::Timer timer_;

        // some parameters
        float cur_vel_;
        float cur_vel_actual_;
        float cur_acc_;
        float cur_latitude_;
        float cur_longitude_;

        // some parameters from yaml
        std::string map_frame_id_;
        float acc_lat_max_;
        float acc_lon_max_;
        float dec_lon_max_;
        float jerk_lon_max_;
        float vel_max_;
        float epsilon_;
        float curvature_redundancy_;
        bool enable_debug_;
        bool speed_update_detect_;
        int path_sub_buffer_;
        int speed_sub_buffer_;
        int traj_pub_buffer_;
        float cal_interval_;
        bool verbose_mode_;
        int debug_pub_buffer_;
        int least_point_num_;

        // nodelet init
        virtual void onInit();

        // Functions
        void initParameters();

        void printParameters();

        // rosmsg callback funtions
//        void pathSubCb(const nav_msgs::Path msg);
        void pathSubCb(const plan2control_msgs::Trajectory msg);

        void speedCb(const control_msgs::GetECUReport msg);

        void accCb(const sensor_msgs::Imu msg);

        void timerCb();

        void gpsCb(const sensor_driver_msgs::GpswithHeading msg);

        // calculate constrained speed functions
        std::vector<double> get_curvature(const std::vector<double> &point_x, const std::vector<double> &point_y,
                                          const unsigned long &points_num, std::vector<bool> *sign);

        std::vector<double> get_arclength(const std::vector<double> &point_x, const std::vector<double> &point_y,
                                          const unsigned long &points_num);

        std::vector<double> limit_lat_acc(const std::vector<double> &v_in, const unsigned int &points_num,
                                          const std::vector<double> &curvature,
                                          const std::vector<bool> &curvature_sign,
                                          const double &acc_lat_max);

        std::vector<double> limit_lon_acc(const std::vector<double> &v_in, const unsigned int &points_num,
                                          const std::vector<double> &s, const double &acc_lon_max);

        std::vector<double> limit_lon_adc(const std::vector<double> &v_in, const unsigned int &points_num,
                                          const std::vector<double> &s, const double &dec_lon_max);

        std::vector<double> limit_lon_jerk(const std::vector<double> &v_in, const unsigned int &points_num,
                                           const std::vector<double> &s, const double &jerk_lon_max);

        double
        cal_proximity(const std::vector<double> &v0, const std::vector<double> &vf, const unsigned int &points_num);
    };
}

#endif //CONSTRAINED_SPEED_NODELET_CONSTRAINED_SPEED_NODELET_H
