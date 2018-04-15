#include "ConstrainedSpeedNodelet.hpp"

namespace constrained_speed {
    void ConstrainedSpeedNodelet::onInit() {
        this->nh_ = getNodeHandle();
        this->private_nh_ = getPrivateNodeHandle();
        initParameters();
        printParameters();
        this->path_sub_ = this->nh_.subscribe("/global_path/traj_plan", this->path_sub_buffer_,
                                              &ConstrainedSpeedNodelet::pathSubCb, this);
        this->speed_sub_ = this->nh_.subscribe("/ecudatareport", this->speed_sub_buffer_,
                                               &ConstrainedSpeedNodelet::speedCb, this);
        this->traj_pub_ = this->nh_.advertise<plan2control_msgs::Trajectory>("/trajectory", this->traj_pub_buffer_);
        this->timer_ = this->nh_.createTimer(ros::Duration(this->cal_interval_),
                                             boost::bind(&ConstrainedSpeedNodelet::timerCb, this));
        if (this->enable_debug_) {
            //// The current acceleration is not used now, but maybe useful later.
            //// this->acc_sub_ = this->nh_.subscribe("/imudata", 1, &ConstrainedSpeedNodelet::accCb, this);

            this->gps_sub_ = this->nh_.subscribe("/gpsdata", 1, &ConstrainedSpeedNodelet::gpsCb, this);
            this->debug_pub_ = this->nh_.advertise<speed_debug_msgs::speed_debug>("/speed_debug",
                                                                                  this->debug_pub_buffer_);
        }
    }

    void ConstrainedSpeedNodelet::initParameters() {
        auto &tmp_nh = this->private_nh_;
        tmp_nh.getParam("map_frame_id", this->map_frame_id_);
        tmp_nh.getParam("v_max", this->vel_max_);
        tmp_nh.getParam("acc_lat_max", this->acc_lat_max_);
        tmp_nh.getParam("acc_lon_max", this->acc_lon_max_);
        tmp_nh.getParam("dec_lon_max", this->dec_lon_max_);
        tmp_nh.getParam("jerk_lon_max", this->jerk_lon_max_);
        tmp_nh.getParam("epsilon", this->epsilon_);
        tmp_nh.getParam("curvature_redundancy", this->curvature_redundancy_);
        tmp_nh.getParam("enable_debug", this->enable_debug_);
        tmp_nh.getParam("speed_update_detect", this->speed_update_detect_);
        tmp_nh.getParam("path_sub_buffer", this->path_sub_buffer_);
        tmp_nh.getParam("speed_sub_buffer", this->speed_sub_buffer_);
        tmp_nh.getParam("traj_pub_buffer", this->traj_pub_buffer_);
        tmp_nh.getParam("cal_interval", this->cal_interval_);
        tmp_nh.getParam("verbose_mode", this->verbose_mode_);
        tmp_nh.getParam("debug_pub_buffer", this->debug_pub_buffer_);
        tmp_nh.getParam("least_point_num", this->least_point_num_);

        this->is_path_update_ = false;
        this->is_speed_update_ = false;
    }

    void ConstrainedSpeedNodelet::printParameters() {
        if (this->verbose_mode_) {
            NODELET_INFO_STREAM("Loaded parameters with namespace "
                                        << this->private_nh_.getNamespace());
            NODELET_INFO_STREAM("map_frame_id " << this->map_frame_id_);
            NODELET_INFO_STREAM("v_max " << this->vel_max_);
            NODELET_INFO_STREAM("acc_lat_max " << this->acc_lat_max_);
            NODELET_INFO_STREAM("acc_lon_max " << this->acc_lon_max_);
            NODELET_INFO_STREAM("dec_lon_max " << this->dec_lon_max_);
            NODELET_INFO_STREAM("jerk_lon_max " << this->jerk_lon_max_);
            NODELET_INFO_STREAM("epsilon " << this->epsilon_);
            NODELET_INFO_STREAM("curvature_redundancy " << this->curvature_redundancy_);
            NODELET_INFO_STREAM("enable_debug " << this->enable_debug_);
            NODELET_INFO_STREAM("speed_update_detect" << this->speed_update_detect_);
            NODELET_INFO_STREAM("path_sub_buffer" << this->path_sub_buffer_);
            NODELET_INFO_STREAM("speed_sub_buffer" << this->speed_sub_buffer_);
            NODELET_INFO_STREAM("traj_pub_buffer" << this->traj_pub_buffer_);
            NODELET_INFO_STREAM("cal_interval" << this->cal_interval_);
            NODELET_INFO_STREAM("verbose_mode" << this->verbose_mode_);
            NODELET_INFO_STREAM("debug_pub_buffer" << this->debug_pub_buffer_);
            NODELET_INFO_STREAM("least_point_num" << this->least_point_num_);
        }

    }

    void ConstrainedSpeedNodelet::timerCb() {
        if (this->verbose_mode_) {
            NODELET_INFO_STREAM("constrained speed: timer callback");
        }
        if (!is_path_update_) {
            if (this->verbose_mode_) {
                NODELET_INFO_STREAM("constrained speed: path not update yet");
            }
            return;
        }
        this->v_max_.clear();
        this->points_num_ = this->trajectory_.points.size();
        if (this->verbose_mode_) {
            NODELET_INFO_STREAM("constrained speed: point num is " << this->points_num_);
        }
        if (this->points_num_ < this->least_point_num_) {
            NODELET_INFO_STREAM("constrained speed: point num incorrect");
            return;
        }
        this->v_max_ = std::vector<double>(this->points_num_, this->vel_max_);
        if (this->speed_update_detect_) {
            if (!is_speed_update_) {
                if (this->verbose_mode_) {
                    NODELET_INFO_STREAM("constrained speed: speed not update yet");
                }
                return;
            }
            this->v_max_[0] = this->cur_vel_;
        }
        auto start = hmpl::now();
        this->v0_.clear();
        this->vf_.clear();
        this->curvature_.clear();
        this->curvature_sign_.clear();
        this->s_.clear();
        this->v_limit_lat_acc_.clear();
        this->v_limit_lon_acc_.clear();
        this->v_limit_lon_adc_.clear();
        this->v_limit_lon_jerk_.clear();
        this->point_x_.clear();
        this->point_y_.clear();


        for (unsigned int i = 0; i < this->points_num_; ++i) {
            this->point_x_.push_back(this->trajectory_.points[i].position.x);
            this->point_y_.push_back(this->trajectory_.points[i].position.y);
        }


        this->curvature_ = get_curvature(this->point_x_, this->point_y_, this->points_num_, &this->curvature_sign_);
        this->s_ = get_arclength(this->point_x_, this->point_y_, this->points_num_);

        this->v_limit_lat_acc_ = limit_lat_acc(this->v_max_, this->points_num_, this->curvature_, this->curvature_sign_,
                                               this->acc_lat_max_);
        this->v_limit_lon_acc_ = limit_lon_acc(this->v_limit_lat_acc_, this->points_num_, this->s_, this->acc_lon_max_);
        this->v_limit_lon_adc_ = limit_lon_adc(this->v_limit_lon_acc_, this->points_num_, this->s_, this->dec_lon_max_);
        this->v_limit_lon_jerk_ = limit_lon_jerk(this->v_limit_lon_adc_, this->points_num_, this->s_,
                                                 this->jerk_lon_max_);

        this->v0_ = this->v_max_;
        this->vf_ = this->v_limit_lon_jerk_;

        unsigned int ir_counter = 1;

        while (cal_proximity(this->v0_, this->vf_, this->points_num_) > (this->epsilon_ * this->points_num_)) {
            this->v0_ = this->vf_;
            for (unsigned int i = 0; i < this->points_num_; ++i) {
                if (this->vf_[i] > this->v_limit_lat_acc_[i]) {
                    this->vf_[i] = this->v_limit_lat_acc_[i];
                }
            }
            this->vf_ = limit_lon_acc(this->vf_, this->points_num_, this->s_, this->acc_lon_max_);
            this->vf_ = limit_lon_adc(this->vf_, this->points_num_, this->s_, this->dec_lon_max_);
            this->vf_ = limit_lon_jerk(this->vf_, this->points_num_, this->s_, this->jerk_lon_max_);

            ++ir_counter;
        }


        for (unsigned int i = 0; i < this->points_num_; ++i) {
            this->trajectory_.points[i].curvature = this->curvature_[i];
            this->trajectory_.points[i].velocity.linear.x = this->vf_[i];
        }

        this->traj_pub_.publish(this->trajectory_);
        this->is_path_update_ = false;
        this->is_speed_update_ = false;
        auto end = hmpl::now();
        double duration_time = hmpl::getDurationInSecs(start, end);
        if (this->enable_debug_) {
            this->speed_debug_.trajectory = trajectory_;
            this->speed_debug_.s = s_;
            this->speed_debug_.vel_max = this->v_max_;
            this->speed_debug_.vel_limit_lat_acc = this->v_limit_lat_acc_;
            this->speed_debug_.vel_limit_lon_acc = this->v_limit_lon_acc_;
            this->speed_debug_.vel_limit_lon_adc = this->v_limit_lon_adc_;
            this->speed_debug_.ir_counter = ir_counter;
            this->speed_debug_.duration_time = duration_time;
            this->speed_debug_.cur_vel = this->cur_vel_;
            this->speed_debug_.cur_vel_actual = this->cur_vel_actual_;
            this->speed_debug_.cur_latitude = this->cur_latitude_;
            this->speed_debug_.cur_longitude = this->cur_longitude_;
            this->debug_pub_.publish(this->speed_debug_);
        }
        if (this->verbose_mode_) {
            NODELET_INFO_STREAM("The iteration times is " << ir_counter);
            NODELET_INFO_STREAM("the cal time is " << duration_time);
        }
    }

    void ConstrainedSpeedNodelet::pathSubCb(const plan2control_msgs::Trajectory msg) {
        if (this->verbose_mode_) {
            NODELET_INFO_STREAM("constrained speed: get a path msg");
        }
        if (this->map_frame_id_ == msg.header.frame_id) {
            this->trajectory_.points.clear();
            this->trajectory_ = msg;
            this->is_path_update_ = true;
            if (this->verbose_mode_) {
                NODELET_INFO_STREAM("constrained speed: frame id correct");
                NODELET_INFO_STREAM("constrained speed: path msg handled");
            }
        } else {
            NODELET_INFO_STREAM("the frame_id not compatible with which in constrained_speed_config.yaml");
            NODELET_INFO_STREAM("the frame_id is " << msg.header.frame_id);
        }
    }


    void ConstrainedSpeedNodelet::speedCb(const control_msgs::GetECUReport msg) {
        this->cur_vel_actual_ = msg.speed.velocity.linear.x;
        this->cur_vel_ = this->cur_vel_actual_ > this->vel_max_? vel_max_: cur_vel_actual_;
        this->is_speed_update_ = true;
        if (this->verbose_mode_) {
            NODELET_INFO_STREAM("constrained speed: current speed updated");
        }
    }

    void ConstrainedSpeedNodelet::accCb(const sensor_msgs::Imu msg) {
        this->cur_acc_ = msg.linear_acceleration.y;
    }

    void ConstrainedSpeedNodelet::gpsCb(const sensor_driver_msgs::GpswithHeading msg) {
        this->cur_latitude_ = msg.gps.latitude;
        this->cur_longitude_ = msg.gps.longitude;
    }


    std::vector<double>
    ConstrainedSpeedNodelet::get_curvature(const std::vector<double> &point_x, const std::vector<double> &point_y,
                                           const unsigned long &points_num, std::vector<bool> *sign) {
        std::vector<double> curvature;
        curvature.push_back(0.0);
        (*sign).push_back(true);
        for (unsigned int i = 1; i < points_num - 1; ++i) {
            double a, b, c;
            double delta_x, delta_y;
            double s;
            double k;
            double curv;
            double rotate_direction;

            delta_x = point_x[i] - point_x[i - 1];
            delta_y = point_y[i] - point_y[i - 1];
            a = sqrt(pow(delta_x, 2.0) + pow(delta_y, 2.0));

            delta_x = point_x[i + 1] - point_x[i];
            delta_y = point_y[i + 1] - point_y[i];
            b = sqrt(pow(delta_x, 2.0) + pow(delta_y, 2.0));

            delta_x = point_x[i - 1] - point_x[i + 1];
            delta_y = point_y[i - 1] - point_y[i + 1];
            c = sqrt(pow(delta_x, 2.0) + pow(delta_y, 2.0));

            s = (a + b + c) / 2.0;
            k = sqrt(fabs(s * (s - a) * (s - b) * (s - c)));
            curv = 4 * k / (a * b * c);

            if (curv < curvature_redundancy_) {
                (*sign).push_back(true);
            } else {
                (*sign).push_back(false);
            }

            rotate_direction = (point_x[i] - point_x[i - 1]) * (point_x[i + 1] - point_x[i]) -
                               (point_y[i] - point_y[i - 1]) * (point_y[i + 1] - point_y[i]);

            if (rotate_direction < 0) {
                curv = -curv;
            }

            curvature.push_back(curv);
        }
        curvature.push_back(*(curvature.end() - 1));
        (*sign).push_back(*((*sign).end() - 1));
        curvature[0] = curvature[1];
        (*sign)[0] = (*sign)[1];
        return curvature;
    }

    std::vector<double>
    ConstrainedSpeedNodelet::get_arclength(const std::vector<double> &point_x, const std::vector<double> &point_y,
                                           const unsigned long &points_num) {
        std::vector<double> s;
        double tmp1;
        tmp1 = 0.0;
        s.push_back(0.0);
        for (unsigned int i = 1; i < points_num; ++i) {
            tmp1 += sqrt(pow((point_x[i] - point_x[i - 1]), 2.0) + pow((point_y[i] - point_y[i - 1]), 2.0));
            s.push_back(tmp1);
        }
        return s;
    }

    std::vector<double>
    ConstrainedSpeedNodelet::limit_lat_acc(const std::vector<double> &v_in, const unsigned int &points_num,
                                           const std::vector<double> &curvature,
                                           const std::vector<bool> &curvature_sign,
                                           const double &acc_lat_max) {

        std::vector<double> v_out;
        for (unsigned int i = 0; i < points_num; i++) {
            if (curvature_sign[i]) {
                v_out.push_back(v_in[i]);
            } else {
                v_out.push_back(std::min(sqrt(acc_lat_max / fabs(curvature[i])), v_in[i]));
            }
        }
        return v_out;
    }

    std::vector<double>
    ConstrainedSpeedNodelet::limit_lon_acc(const std::vector<double> &v_in, const unsigned int &points_num,
                                           const std::vector<double> &s, const double &acc_lon_max) {
        std::vector<double> v_out;
        v_out.push_back(v_in[0]);
        for (unsigned long i = 1; i < points_num; i++) {
            v_out.push_back(std::min(sqrt(pow(v_out[i - 1], 2.0) + 2.0 * acc_lon_max * (s[i] - s[i - 1])), v_in[i]));
        }
        return v_out;
    }

    std::vector<double>
    ConstrainedSpeedNodelet::limit_lon_adc(const std::vector<double> &v_in, const unsigned int &points_num,
                                           const std::vector<double> &s, const double &dec_lon_max) {
        std::vector<double> v_out(v_in);
        for (unsigned long i = points_num - 1; i > 0; i--) {
            v_out[i - 1] = std::min(sqrt(pow(v_out[i], 2.0) + 2.0 * dec_lon_max * (s[i] - s[i - 1])), v_in[i - 1]);
        }
        return v_out;
    }

    std::vector<double>
    ConstrainedSpeedNodelet::limit_lon_jerk(const std::vector<double> &v_in, const unsigned int &points_num,
                                            const std::vector<double> &s, const double &jerk_lon_max) {
        std::vector<double> v_out(v_in);
        double tmpss1;
        double tmpss2;
        double tmpss3;
        double tmpss4;
        double tmpss5;
        double tmpss6;
        double tmp7;
        double tmp8;
        double tmp9;
        double tmp10;
        double jerk;
        for (unsigned long i = 1; i < points_num - 1; i++) {
            tmpss1 = s[i - 1] * s[i];
            tmpss2 = s[i - 1] * s[i + 1];
            tmpss3 = s[i] * s[i + 1];
            tmpss4 = pow(s[i - 1], 2.0);
            tmpss5 = pow(s[i], 2.0);
            tmpss6 = pow(s[i + 1], 2.0);
            tmp7 = (tmpss1 - tmpss2 + tmpss3 - tmpss5) / 2.0;
            tmp8 = 2.0 * v_out[i + 1] / (tmpss1 - tmpss2 - tmpss3 + tmpss6);
            tmp9 = 2.0 * v_out[i - 1] / (tmpss1 + tmpss2 - tmpss3 - tmpss4);
            tmp10 = 2.0 * v_out[i] / (tmpss1 - tmpss2 + tmpss3 - tmpss5);
            jerk = tmp8 - tmp9 - tmp10;
            if (jerk > jerk_lon_max) {
                v_out[i] = (tmp8 - tmp9 - jerk_lon_max) * tmp7;
            }
            if (jerk < (-jerk_lon_max)) {
                v_out[i] = (tmp8 - tmp9 + jerk_lon_max) * tmp7;
            }
        }
        return v_out;
    }

    double ConstrainedSpeedNodelet::cal_proximity(const std::vector<double> &v0, const std::vector<double> &vf,
                                                  const unsigned int &points_num) {
        double tmp1;
        tmp1 = 0.0;
        for (unsigned long i = 0; i < points_num; i++) {
            tmp1 += fabs(v0[i] - vf[i]);
        }
        return tmp1;
    }

}
PLUGINLIB_DECLARE_CLASS(constrained_speed, ConstrainedSpeedNodelet,
                        constrained_speed::ConstrainedSpeedNodelet,
                        nodelet::Nodelet)
//PLUGINLIB_EXPORT_CLASS(constrained_speed::ConstrainedSpeedNodelet, nodelet::Nodelet)
