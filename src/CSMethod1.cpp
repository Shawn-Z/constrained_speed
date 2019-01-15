#include <CSMethod1.hpp>
#include <Points_Attributes.hpp>

namespace constrained_speed {

CSMethod1::CSMethod1(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
    : reconfigSrv_{private_node_handle},
      params_{private_node_handle} {
    this->nh_ = node_handle;
    this->private_nh_ = private_node_handle;

//    cs_method1Config default_config_;
//    this->reconfigSrv_.getConfigDefault(default_config_);
//    this->params_.fromConfig(default_config_);
//    this->params_.toParamServer();
//    this->reconfigSrv_.setConfigDefault(default_config_);

    this->params_.fromParamServer();
//    ROS_INFO_STREAM(this->params_);

    //// todo, init something that constant not change or have to
    init();

    if (this->params_.reconfig) {
        this->reconfigSrv_.setCallback(boost::bind(&CSMethod1::reconfigureRequest, this, _1, _2));
    }

    std::string way_topic = this->private_nh_.param<std::string>("road_net_topic", "/topology_global_path");

    if (this->super_points_.on_off.p_issue_mode == this->params_.issue_mode_cycle) {
        this->run_timer_ = this->nh_.createTimer(ros::Duration(0.02), boost::bind(&CSMethod1::timerCb, this));
    }
    this->check_timer_ = this->nh_.createTimer(ros::Duration(0.15), boost::bind(&CSMethod1::time_check, this));

    //// Publish
    this->traj_pub_ = this->nh_.advertise<plan2control_msgs::Trajectory>("/trajectory", 1);
    this->ctrl_pub_ = this->nh_.advertise<speed_ctrl_msgs::speed_ctrl>("/speed_plan", 1);
    this->debug_pub_ = this->nh_.advertise<speed_debug_msgs::speed_debug>("/speed_debug", 1);

    //// Subscribe
    this->path_sub_ = this->nh_.subscribe("/global_path/traj_plan", 1, &CSMethod1::pathSubCb, this);
    this->collision_sub_ = this->nh_.subscribe("/dymi_point_to_speed", 1, &CSMethod1::collisionCb, this);
//    this->ramp_sub_ = this->nh_.subscribe("/gpsdata", 1, &CSMethod1::rampCb, this);
    this->segmentRoad_sub_ = this->nh_.subscribe(way_topic, 1, &CSMethod1::segmentRoadCb, this);
#ifdef TOYOTA
    this->speed_sub_ = this->nh_.subscribe("/ecudatareport", 1, &CSMethod1::speedCb, this);
#endif
#ifdef NORTH_COLA
    this->speed_sub_ = this->nh_.subscribe("/ecudatareport", 1, &CSMethod1::speedCb, this);
#endif
#ifdef TANK_6T
    this->speed_sub_ = this->nh_.subscribe("/ecudatareport", 1, &CSMethod1::speedCb, this);
#endif
}

void CSMethod1::pathSubCb(const plan2control_msgs::Trajectory msg) {
    if ((msg.header.frame_id != "base_link") && (msg.header.frame_id != "search_base_link")) {
        ROS_ERROR_STREAM("CS: frame-id wrong, receive: " << msg.header.frame_id);
        return;
    }

    if ((msg.points.size() < 2) || (msg.points.size() > 1000)) {
        ROS_ERROR_STREAM("CS: points num is " << msg.points.size() << ", illegal, return");
        return;
    }

    this->super_points_.time_check.rc_ros_time = ros::Time::now().toSec() * 1000;
    if (this->super_points_.on_off.p_verbose) {
        ROS_INFO_STREAM("CS: pathSubCb");
    }
    this->trajectory_ = msg;
    this->super_points_.msg_udt.update.items.path = true;

    if (this->super_points_.on_off.p_issue_mode == this->params_.issue_mode_direct) {
        timerCb();
    }
}

void CSMethod1::collisionCb(const dymidetect2speed_msgs::dymicol_point msg) {
    if (this->super_points_.on_off.p_verbose) {
        ROS_INFO_STREAM("CS: collisionCb");
    }
    this->dymicol_point_ = msg;
    this->super_points_.msg_udt.addition.collision = true;
}

void CSMethod1::rampCb(const sensor_driver_msgs::GpswithHeading msg) {
    if (this->super_points_.on_off.p_verbose) {
        ROS_INFO_STREAM("CS: rampCb");
    }
    this->gpswithHeading_ = msg;
    this->super_points_.msg_udt.addition.ramp = true;
}

void CSMethod1::segmentRoadCb(const lanelet_map_msgs::Way msg) {
    this->way_ = msg;
    this->super_points_.time_check.rc_way_time = ros::Time::now().toSec() * 1000;
    //// todo  msg.type
}

void CSMethod1::init() {
    if (this->super_points_.on_off.p_verbose) {
        ROS_WARN_STREAM("CS: init");
    }

    this->super_points_.cur_speed.seq.clear();
    this->super_points_.cur_speed.seq.assign(this->params_.speed_seq_num, 0);

    this->super_points_.cur_steer.seq.clear();
    this->super_points_.cur_steer.seq.assign(this->params_.steer_seq_num, 0);

    this->super_points_.brake_amount.acc_times.clear();
    this->super_points_.brake_amount.dec_times.clear();
    this->super_points_.brake_amount.decs.clear();

    this->trajectory_.header.frame_id = "tmp";

    this->super_points_.msg_udt.yes = 3;
    this->super_points_.msg_udt.update.result = 0;
    this->super_points_.msg_udt.addition.collision = false;
    this->super_points_.msg_udt.addition.ramp = false;

    this->super_points_.direction = gear_type::N;
    this->super_points_.cur_direction = gear_type::N;

    this->super_points_.cycle_count = 0;

    this->gear_strings_[gear_type::P] = "P";
    this->gear_strings_[gear_type::N] = "N";
    this->gear_strings_[gear_type::R] = "R";
    this->gear_strings_[gear_type::D1] = "D1";
    this->gear_strings_[gear_type::D2] = "D2";
    this->gear_strings_[gear_type::D3] = "D3";
    this->gear_strings_[gear_type::D4] = "D4";
    this->gear_strings_[gear_type::D5] = "D5";
    this->gear_strings_[gear_type::unknown] = "unknown";

    this->running_mode_strings_[running_mode_type::default_mode] = "default_mode";
    this->running_mode_strings_[running_mode_type::general_ahead] = "general_ahead";
    this->running_mode_strings_[running_mode_type::general_retreat] = "general_retreat";
    this->running_mode_strings_[running_mode_type::search_ahead] = "search_ahead";
    this->running_mode_strings_[running_mode_type::search_retreat] = "search_retreat";
    this->running_mode_strings_[running_mode_type::foggy_ahead] = "foggy_ahead";
    this->running_mode_strings_[running_mode_type::foggy_retreat] = "foggy_retreat";
    this->running_mode_strings_[running_mode_type::inspect_ahead] = "inspect_ahead";
    this->running_mode_strings_[running_mode_type::inspect_retreat] = "inspect_retreat";
    this->running_mode_strings_[running_mode_type::dynamic_collision_ahead] = "dynamic_collision_ahead";
    this->running_mode_strings_[running_mode_type::dynamic_collision_retreat] = "dynamic_collision_retreat";
    this->running_mode_strings_[running_mode_type::unknown] = "unknown";

    this->super_points_.dynamic_collision.update_count = 0;

    this->super_points_.ramp.slopes.clear();
    this->super_points_.ramp.slopes.assign(20, 0);
}

void CSMethod1::timerCb() {
    if (this->super_points_.on_off.p_program_halt) {
        ROS_WARN_STREAM("CS: program halt, not pub and cal, return");
        return;
    }

    if (this->super_points_.on_off.p_verbose) {
        ROS_INFO_STREAM("CS: timerCb");
    }

    if ((this->super_points_.msg_udt.update.result) != (this->super_points_.msg_udt.yes)) {
        if (!this->super_points_.msg_udt.update.items.path) {
            ROS_WARN_STREAM("CS: path not update    " << this->super_points_.msg_udt.update.items.path);
        }
        if (!this->super_points_.msg_udt.update.items.speed) {
            ROS_WARN_STREAM("CS: speed not update    " << this->super_points_.msg_udt.update.items.speed);
        }
//        state_reset_after();
        return;
    }

    state_reset_before();
    dynamic_collision();
    decide_running_mode();

    if (this->super_points_.running_mode == running_mode_type::unknown) {
        ROS_INFO_STREAM("CS: unknown running mode, return");
        return;
    }

    points_attributes();

    if (this->super_points_.error_mark.update.result != 0) {
        if (this->super_points_.error_mark.update.items.same_direction_points_num) {
            ROS_ERROR_STREAM("CS: same direction points num is " << this->super_points_.point_num << ", illegal, return");
        }
        if (this->super_points_.error_mark.update.items.pointsAttributes_calculate) {
            if (this->pointsAttributes_.error.update.items.s_damn_short) {
                ROS_INFO_STREAM("s_damn_short");
            }
            if (this->pointsAttributes_.error.update.items.points_empty) {
                ROS_INFO_STREAM("points_empty");
            }
            if (this->pointsAttributes_.error.update.items.system_size_over) {
                ROS_INFO_STREAM("system_size_over");
            }
            if (this->pointsAttributes_.error.update.items.points_num_unequal) {
                ROS_INFO_STREAM("points_num_unequal");
            }
            if (this->pointsAttributes_.error.update.items.points_num_less) {
                ROS_INFO_STREAM("points_num_less");
            }
            if (this->pointsAttributes_.error.update.items.points_num_over) {
                ROS_INFO_STREAM("points_num_over");
            }
            ROS_ERROR_STREAM("CS: something wrong when pointsAttributes.calculate, return");
        }
        if (this->super_points_.error_mark.update.items.reach_safe_distance) {
            ROS_ERROR_STREAM("CS: reach safe distance, return");
        }
        if (this->super_points_.error_mark.update.items.direction_change) {
            ROS_ERROR_STREAM("CS: direction change, return");
        }
        if (this->super_points_.error_mark.update.items.desired_direction_wrong) {
            ROS_ERROR_STREAM("CS: desired direction wrong, return");
        }
        return;
    }

    general_planning();
    state_reset_after();

    ROS_INFO_STREAM("CS: time_duration    " << (this->super_points_.time_check.pub_ros_time - this->super_points_.time_check.rc_ros_time));
}


void CSMethod1::state_reset_before() {
    if (this->super_points_.on_off.p_verbose) {
        ROS_INFO_STREAM("CS: state_reset_before");
    }

    this->super_points_.msg_udt.update.result = 0;
    this->super_points_.error_mark.update.result = 0;

    this->super_points_.slide_dec.slide_gen_warn = false;
    this->super_points_.lon_dec.dec_gen_warn = false;
    this->super_points_.dec_fail.dec_fail_gen_warn = false;
    this->super_points_.dec_fail.dec_fail_gen_warn2 = false;

    this->super_points_.warn_diff_direction = false;
}

void CSMethod1::state_reset_after() {
    if (this->super_points_.on_off.p_verbose) {
        ROS_INFO_STREAM("CS: state_reset_after");
    }

    this->super_points_.msg_udt.update.result = 0;
    this->super_points_.error_mark.update.result = 0;

    this->super_points_.msg_udt.addition.collision = false;
    this->super_points_.msg_udt.addition.ramp = false;

    this->super_points_.slide_dec.slide_gen_warn = false;
    this->super_points_.lon_dec.dec_gen_warn = false;
    this->super_points_.dec_fail.dec_fail_gen_warn = false;
    this->super_points_.dec_fail.dec_fail_gen_warn2 = false;

    this->super_points_.warn_diff_direction = false;
}

void CSMethod1::points_attributes() {
    if (this->super_points_.on_off.p_verbose) {
        ROS_INFO_STREAM("CS: points_attributes");
    }

    switch (this->trajectory_.points[0].forward) {
        case 0: {
            this->super_points_.direction = gear_type::R;
            break;
        }
        case 1: {
            this->super_points_.direction = gear_type::D1;
            break;
        }
        default: {
            this->super_points_.direction = gear_type::unknown;
            this->super_points_.error_mark.update.items.desired_direction_wrong = true;
            return;
            break;
        }
    }

#ifdef TANK_6T
    if ((this->super_points_.cur_direction == gear_type::D1) || (this->super_points_.cur_direction == gear_type::R)) {
        if (this->super_points_.cur_direction != this->super_points_.direction) {
            this->super_points_.error_mark.update.items.direction_change = true;
            return;
        }
    }
#endif

    this->super_points_.point_num = count_type(this->trajectory_.points.size());

    this->super_points_.x_points.clear();
    this->super_points_.y_points.clear();

    //// check for same direction points
    for (count_type i = 0; i < this->super_points_.point_num; ++i) {
        if (this->trajectory_.points[i].forward != this->trajectory_.points[0].forward) {
            this->super_points_.point_num = i;
            if (this->super_points_.point_num < 2) {
                this->super_points_.error_mark.update.items.same_direction_points_num = true;
                return;
            }
            break;
        }
        this->super_points_.x_points.emplace_back(this->trajectory_.points[i].position.x);
        this->super_points_.y_points.emplace_back(this->trajectory_.points[i].position.y);
    }

    this->super_points_.warn_diff_direction = this->super_points_.point_num != this->trajectory_.points.size();

    if (this->super_points_.point_num == 2) {
        this->super_points_.tmp_vel_data1.clear();
        this->super_points_.tmp_vel_data2.clear();
        this->super_points_.tmp_vel_data1.emplace_back(this->super_points_.x_points[0]);
        this->super_points_.tmp_vel_data1.emplace_back(0.5 * (this->super_points_.x_points[0] + this->super_points_.x_points[1]));
        this->super_points_.tmp_vel_data1.emplace_back(this->super_points_.x_points[1]);
        this->super_points_.tmp_vel_data2.emplace_back(this->super_points_.y_points[0]);
        this->super_points_.tmp_vel_data2.emplace_back(0.5 * (this->super_points_.y_points[0] + this->super_points_.y_points[1]));
        this->super_points_.tmp_vel_data2.emplace_back(this->super_points_.y_points[1]);
        this->super_points_.x_points.clear();
        this->super_points_.y_points.clear();
        this->super_points_.x_points = this->super_points_.tmp_vel_data1;
        this->super_points_.y_points = this->super_points_.tmp_vel_data2;
    }
    //// todo new dian xishujiancha
    //// todo edit
    this->point_attributes_params_.curvature_max = this->params_.curvature_max;
    this->point_attributes_params_.half_s_curv_cal = this->params_.half_s_curv_cal;
    this->point_attributes_params_.min_total_length = 0.01;
    this->point_attributes_params_.curv_zero_redundancy = 0.001;
    this->point_attributes_params_.max_size = 10000;
    this->point_attributes_params_.min_size = 3;
    //// todo
    this->pointsAttributes_.calculate(this->point_attributes_params_, this->super_points_.x_points, this->super_points_.y_points);
    if (this->pointsAttributes_.error.update.result != 0) {
        this->super_points_.error_mark.update.items.pointsAttributes_calculate = true;
        return;
    }

    this->super_points_.s.clear();
    this->super_points_.interval.clear();
    this->super_points_.curv_zero_sign.clear();
    this->super_points_.curv_final.clear();

    this->super_points_.s = this->pointsAttributes_.s;
    this->super_points_.assist_params.total_arc_length = this->super_points_.s.back();
    this->super_points_.interval = this->pointsAttributes_.interval;
    this->super_points_.curv_zero_sign = this->pointsAttributes_.curv_cal.curv_zero_sign;
    this->super_points_.curv_final = this->pointsAttributes_.curv_cal.curv_final;

    this->super_points_.v_constrained.clear();
    this->super_points_.v_constrained.assign(this->super_points_.point_num, 20);

    cal_assist_params();
    if (this->super_points_.assist_params.total_arc_length <= this->super_points_.assist_params.smart_safe_distance) {
        this->super_points_.error_mark.update.items.reach_safe_distance = true;
        return;
    }
}

void CSMethod1::general_planning() {
    if (this->super_points_.on_off.p_verbose) {
        ROS_INFO_STREAM("CS: general_planning");
    }

#ifdef TOYOTA
    fake_state();
//    dynamic_collision();
    durex();
    limit_segmentRoad();
    set_v_max();
    limit_by_steer();
//    limit_by_ramp();
    limit_lat_acc();
    blind_handle();
    limit_lon_acc();
    limit_slide_dec();
    limit_lon_dec();
    dec_fail_handle();
    limit_jerk();
    find_first_min_v();
    cal_acc();
    cal_time();
    issue_cal();
    brake_amount();
    fake_issue();
    publish();
    debug_handle();

    if (this->super_points_.on_off.p_verbose) {
        ROS_INFO_STREAM("speed information");
//        for (count_type i = 0; i < this->super_points_.point_num; ++i) {
//            ROS_WARN("index: %d  %s: %f  %s: %f  %s: %f  %s: %f  %s: %f  %s: %f  %s: %f  %s: %f  %s: %f"
//                     , i
//            , "constrained", this->super_points_.v_constrained[i]
//            , "v_max_dynamic", this->super_points_.set_v_max.v_max_dynamic[i]
//            , "v_lat_acc", this->super_points_.lat_acc.v_limit[i]
//            , "v_blind", this->super_points_.blind.v_blind[i]
////            , "v_collision", this->super_points_.dynamic_collision.v_limit[i]
//            , "v_lon_acc", this->super_points_.lon_acc.v_limit[i]
//            , "v_slide_dec", this->super_points_.slide_dec.v_limit[i]
//            , "v_lon_dec", this->super_points_.lon_dec.v_limit[i]
//            , "v_jerk", this->super_points_.jerk.v_limit[i]
//            , "acc", this->super_points_.acc[i]
//            );
//        }
//        ROS_INFO_STREAM("CS: some limit info: safe: " << this->super_points_.durex.v_limit << " steer: " << this->super_points_.steer.v_limit << " ramp: " << this->super_points_.ramp.v_limit);
        ROS_INFO_STREAM(" ");
    }
#endif
#ifdef NORTH_COLA
    fake_state();
//    dynamic_collision();
    durex();
    limit_segmentRoad();
    set_v_max();
    limit_by_steer();
//    limit_by_ramp();
    limit_lat_acc();
    blind_handle();
    limit_lon_acc();
    limit_slide_dec();
    limit_lon_dec();
    dec_fail_handle();
    limit_jerk();
    find_first_min_v();
    cal_acc();
    cal_time();
    issue_cal();
    brake_amount();
    fake_issue();
    publish();
    debug_handle();

    if (this->super_points_.on_off.p_verbose) {
        ROS_INFO_STREAM("speed information");
//        for (count_type i = 0; i < this->super_points_.point_num; ++i) {
//            ROS_WARN("index: %d  %s: %f  %s: %f  %s: %f  %s: %f  %s: %f  %s: %f  %s: %f  %s: %f  %s: %f"
//                     , i
//            , "constrained", this->super_points_.v_constrained[i]
//            , "v_max_dynamic", this->super_points_.set_v_max.v_max_dynamic[i]
//            , "v_lat_acc", this->super_points_.lat_acc.v_limit[i]
//            , "v_blind", this->super_points_.blind.v_blind[i]
////            , "v_collision", this->super_points_.dynamic_collision.v_limit[i]
//            , "v_lon_acc", this->super_points_.lon_acc.v_limit[i]
//            , "v_slide_dec", this->super_points_.slide_dec.v_limit[i]
//            , "v_lon_dec", this->super_points_.lon_dec.v_limit[i]
//            , "v_jerk", this->super_points_.jerk.v_limit[i]
//            , "acc", this->super_points_.acc[i]
//            );
//        }
//        ROS_INFO_STREAM("CS: some limit info: safe: " << this->super_points_.durex.v_limit << " steer: " << this->super_points_.steer.v_limit << " ramp: " << this->super_points_.ramp.v_limit);
        ROS_INFO_STREAM(" ");
    }
#endif

#ifdef TANK_6T
    fake_state();
//    dynamic_collision();
    durex();
    set_v_max();
    limit_by_steer();
//    limit_by_ramp();
    limit_lat_acc();
    blind_handle();
    limit_lon_acc();
    limit_slide_dec();
    limit_lon_dec();
    dec_fail_handle();
    limit_jerk();
    find_first_min_v();
    cal_acc();
    cal_time();
    issue_cal();
    fake_issue();
    publish();
    debug_handle();
#endif
}

void CSMethod1::durex() {
    this->super_points_.durex.p_dec_delay_time = this->params_.dec_delay;
    this->super_points_.durex.p_safe_dec = this->params_.safe_dec;
    this->super_points_.durex.p_safe_redundancy = this->super_points_.assist_params.smart_safe_distance;

    //// todo  倒车安全距离想要设为0, 请取消注释下面三行
//    if ((this->super_points_.direction == gear_type::R) && (this->trajectory_.header.frame_id == "/base_link")) {
//        this->super_points_.durex.p_safe_redundancy = 0;
//    }

//    if (this->trajectory_.header.frame_id == "/search_base_link") {
//        this->super_points_.durex.p_safe_redundancy = 0;
//    }

    if (this->super_points_.on_off.p_verbose) {
        ROS_INFO_STREAM("CS: durex");
    }

    this->super_points_.durex.safe_redundancy_index = 0;
    for (count_type i = this->super_points_.point_num - 1; i >= 0; --i) {
        if ((this->super_points_.s.back() - this->super_points_.s[i]) >= this->super_points_.durex.p_safe_redundancy) {
            this->super_points_.durex.safe_redundancy_index = i;
            break;
        }
    }
    this->super_points_.durex.at = fabs(this->super_points_.durex.p_safe_dec * this->super_points_.durex.p_dec_delay_time);
    this->super_points_.durex.effective_s = this->super_points_.s.back() - this->super_points_.durex.p_safe_redundancy;
    if (this->super_points_.durex.effective_s < 0) {this->super_points_.durex.effective_s = 0;}
    this->super_points_.durex.v_limit = sqrt(pow(this->super_points_.durex.at, 2.0) + fabs(2.0 * this->super_points_.durex.p_safe_dec * this->super_points_.durex.effective_s)) - this->super_points_.durex.at;
    for (auto & tmp_v: this->super_points_.v_constrained) {
        if (tmp_v > this->super_points_.durex.v_limit) {
            tmp_v = this->super_points_.durex.v_limit;
        }
    }
    this->super_points_.v_constrained[0] = this->super_points_.cur_speed.speed;
}

void CSMethod1::set_v_max() {
    //// todo params
    this->super_points_.set_v_max.p_v_max_general = this->params_.v_max_general;
    this->super_points_.set_v_max.p_v_max_cycle_diff = this->params_.v_max_cycle_diff;
    this->super_points_.set_v_max.p_v_max_cycle_times = this->params_.v_max_cycle_times;
    this->super_points_.set_v_max.p_slide_dec = this->params_.slide_dec + 0.01;
    this->super_points_.set_v_max.p_jerk_O_S_cave_max = this->params_.jerk_O_S_cave_max;
    this->super_points_.set_v_max.p_jerk_O_S_vex_max = this->params_.jerk_O_S_vex_max;

    if (this->super_points_.set_v_max.p_v_max_general > this->way_.vel_limit) {
        this->super_points_.set_v_max.p_v_max_general = this->way_.vel_limit;
        ROS_INFO_STREAM("CS: roadmap limit: " << this->way_.vel_limit);
    }

//    if (this->super_points_.set_v_max.p_v_max_general > this->super_points_.segmentRoad.v_limit) {
//        this->super_points_.set_v_max.p_v_max_general = this->super_points_.segmentRoad.v_limit;
//        ROS_INFO_STREAM("CS: roadnet cal v limit: " << this->super_points_.segmentRoad.v_limit);
//    }

    /// general_ahead
    if (this->super_points_.running_mode == running_mode_type::general_ahead) {
    }

    /// general_retreat
    if (this->super_points_.running_mode == running_mode_type::general_retreat) {
        if (this->super_points_.set_v_max.p_v_max_general > 1.5) {
            this->super_points_.set_v_max.p_v_max_general = 1.5;
        }
    }

    /// dynamic_collision_ahead
    if (this->super_points_.running_mode == running_mode_type::dynamic_collision_ahead) {
        if (this->super_points_.set_v_max.p_v_max_general < this->dymicol_point_.exp_vel) {
            this->super_points_.set_v_max.p_v_max_general = this->dymicol_point_.exp_vel;
        }
    }

    /// dynamic_collision_retreat
    if (this->super_points_.running_mode == running_mode_type::dynamic_collision_retreat) {
        if (this->super_points_.set_v_max.p_v_max_general < this->dymicol_point_.exp_vel) {
            this->super_points_.set_v_max.p_v_max_general = this->dymicol_point_.exp_vel;
        }
    }

    /// foggy_ahead
    if (this->super_points_.running_mode == running_mode_type::foggy_ahead) {
        if (this->super_points_.set_v_max.p_v_max_general > 1.5) {
            this->super_points_.set_v_max.p_v_max_general = 1.5;
        }
    }

    /// foggy_retreat
    if (this->super_points_.running_mode == running_mode_type::foggy_retreat) {
        if (this->super_points_.set_v_max.p_v_max_general > 1.5) {
            this->super_points_.set_v_max.p_v_max_general = 1.5;
        }
    }

    /// inspect_ahead
    if (this->super_points_.running_mode == running_mode_type::inspect_ahead) {
        if (this->super_points_.set_v_max.p_v_max_general > 1.5) {
            this->super_points_.set_v_max.p_v_max_general = 1.5;
        }
    }

    /// inspect_retreat
    if (this->super_points_.running_mode == running_mode_type::inspect_retreat) {
        if (this->super_points_.set_v_max.p_v_max_general > 1.5) {
            this->super_points_.set_v_max.p_v_max_general = 1.5;
        }
    }

//    /// search_ahead
//    if (this->super_points_.running_mode == running_mode_type::search_ahead) {
//        if (this->super_points_.set_v_max.p_v_max_general > 1.5) {
//            this->super_points_.set_v_max.p_v_max_general = 1.5;
//        }
//    }
//
//    /// search_retreat
//    if (this->super_points_.running_mode == running_mode_type::search_retreat) {
//        if (this->super_points_.set_v_max.p_v_max_general > 1.5) {
//            this->super_points_.set_v_max.p_v_max_general = 1.5;
//        }
//    }
//
//    /// search_direction_diff
//    if (this->super_points_.running_mode == running_mode_type::search_direction_diff) {
//        if (this->super_points_.set_v_max.p_v_max_general > 1.5) {
//            this->super_points_.set_v_max.p_v_max_general = 1.5;
//        }
//    }

    /// search_ahead
    if (this->super_points_.running_mode == running_mode_type::search_ahead) {
        if (this->super_points_.set_v_max.p_v_max_general > this->trajectory_.points[0].velocity.linear.x) {
            this->super_points_.set_v_max.p_v_max_general = this->trajectory_.points[0].velocity.linear.x;
        }
    }

    /// search_retreat
    if (this->super_points_.running_mode == running_mode_type::search_retreat) {
        if (this->super_points_.set_v_max.p_v_max_general > this->trajectory_.points[0].velocity.linear.x) {
            this->super_points_.set_v_max.p_v_max_general = this->trajectory_.points[0].velocity.linear.x;
        }
    }

    /// search_direction_diff
    if (this->super_points_.running_mode == running_mode_type::search_direction_diff) {
        if (this->super_points_.set_v_max.p_v_max_general > this->trajectory_.points[0].velocity.linear.x) {
            this->super_points_.set_v_max.p_v_max_general = this->trajectory_.points[0].velocity.linear.x;
        }
    }

    this->super_points_.set_v_max.v_max_dynamic.clear();
    this->super_points_.set_v_max.v_max_dynamic = this->super_points_.v_constrained;
    this->super_points_.tmp_acc.clear();
    this->super_points_.tmp_acc.assign(this->super_points_.point_num, 0);
    this->super_points_.tmp_acc[0] = this->super_points_.cur_acc.acc;
    this->super_points_.tmp_v.clear();
    this->super_points_.tmp_v.assign(this->super_points_.point_num, 0);

    if (this->super_points_.cur_speed.speed > this->super_points_.set_v_max.p_v_max_general) {
        vel_type v0; vel_type a0; vel_type s; vel_type jerk; vel_type v_up; vel_type v_down; vel_type a_up; vel_type a_down;
        for (count_type i = 1; i < this->super_points_.point_num; ++i) {
            v0 = this->super_points_.set_v_max.v_max_dynamic[i - 1];
            a0 = this->super_points_.tmp_acc[i - 1];
            if (a0 > 0) {a0 = 0;}
            s = this->super_points_.interval[i];
            jerk = this->super_points_.set_v_max.p_jerk_O_S_vex_max;
            v_up = this->super_points_.cur_speed.speed;
            v_down = this->super_points_.set_v_max.p_v_max_general;
            a_up = 0;
            a_down = this->super_points_.set_v_max.p_slide_dec;
            this->super_points_.set_v_max.v_max_dynamic[i] = jerk_speed_solution(v0, a0, s, jerk, v_up, v_down, a_up, a_down);
            this->super_points_.tmp_acc[i] = (pow(this->super_points_.set_v_max.v_max_dynamic[i], 2.0) - pow(this->super_points_.set_v_max.v_max_dynamic[i - 1], 2.0)) * 0.5 / this->super_points_.interval[i];
        }

        uint16_t tmp_cycle_times = 0;
        vel_type tmp_diff1 = 0;
        vel_type tmp_diff2 = 0;
        this->super_points_.tmp_v[0] = this->super_points_.set_v_max.v_max_dynamic[0];
        this->super_points_.tmp_v.back() = this->super_points_.set_v_max.v_max_dynamic.back();
        do {
            ++tmp_cycle_times;
            vel_type v_1; vel_type v1; vel_type s0; vel_type s1; vel_type jerk1;
            vel_type tmp_v;
            for (count_type i = 1; i < this->super_points_.point_num - 1; ++i) {
                this->super_points_.tmp_v[i] = this->super_points_.set_v_max.v_max_dynamic[i];
                v_1 = this->super_points_.set_v_max.v_max_dynamic[i - 1];
                v1 = this->super_points_.set_v_max.v_max_dynamic[i + 1];
                s0 = this->super_points_.interval[i];
                s1 = this->super_points_.interval[i + 1];
                jerk1 = this->super_points_.set_v_max.p_jerk_O_S_cave_max;
                tmp_v = jerk_speed_solution(v_1, v1, s0, s1, jerk1);
                if (this->super_points_.set_v_max.v_max_dynamic[i] < tmp_v) {
                    this->super_points_.set_v_max.v_max_dynamic[i] = tmp_v;
                }
                tmp_diff1 = fabs(this->super_points_.set_v_max.v_max_dynamic[i] - this->super_points_.tmp_v[i]);
                if (tmp_diff2 < tmp_diff1) {
                    tmp_diff2 = tmp_diff1;
                }
            }
        } while ((tmp_diff2 > this->super_points_.set_v_max.p_v_max_cycle_diff) && (tmp_cycle_times < this->super_points_.set_v_max.p_v_max_cycle_times));
    }

    if (this->super_points_.cur_speed.speed <= this->super_points_.set_v_max.p_v_max_general) {
        for (count_type i = 1; i < this->super_points_.point_num; ++i) {
            this->super_points_.set_v_max.v_max_dynamic[i] = this->super_points_.set_v_max.p_v_max_general;
        }
    }

    for (count_type i = 1; i < this->super_points_.point_num; ++i) {
        if (this->super_points_.v_constrained[i] > this->super_points_.set_v_max.v_max_dynamic[i]) {
            this->super_points_.v_constrained[i] = this->super_points_.set_v_max.v_max_dynamic[i];
        }
    }
    if (this->super_points_.v_constrained.back() > this->super_points_.set_v_max.p_v_max_general) {
        this->super_points_.v_constrained.back() = this->super_points_.set_v_max.p_v_max_general;
    }

    if (this->super_points_.set_v_max.p_v_max_general < (this->params_.v_max_general - 0.001)) {
        for (count_type i = 1; i < this->super_points_.point_num; ++i) {
            if (this->super_points_.v_constrained[i] > this->super_points_.set_v_max.p_v_max_general) {
                this->super_points_.v_constrained[i] = this->super_points_.set_v_max.p_v_max_general;
            }
        }
    }

//    for (count_type i = 1; i < this->super_points_.point_num; ++i) {
//        if (this->super_points_.v_constrained[i] > this->super_points_.set_v_max.p_v_max_general) {
//            this->super_points_.v_constrained[i] = this->super_points_.set_v_max.p_v_max_general;
//        }
//    }

    this->super_points_.v_constrained[0] = this->super_points_.cur_speed.speed;
}

void CSMethod1::limit_by_steer() {
    this->super_points_.steer.p_wheelbase = this->params_.wheelbase;
    this->super_points_.steer.p_steer_amplify = this->params_.steer_amplify;
    this->super_points_.steer.p_steer_lat_acc = this->params_.steer_lat_acc;


    if (this->super_points_.on_off.p_verbose) {
        ROS_INFO_STREAM("CS: limit_by_steer");
    }

    if (this->super_points_.cur_steer.fabs_max < 3) {return;}
    if (this->super_points_.cur_steer.fabs_max > 32) {
        this->super_points_.cur_steer.fabs_max = 32;
    }

#ifdef TOYOTA
    this->super_points_.steer.v_limit = sqrt(this->super_points_.steer.p_steer_lat_acc * this->super_points_.steer.p_wheelbase / (tan(this->super_points_.steer.p_steer_amplify * fabs(this->super_points_.cur_steer.fabs_max * acos(-1.0) / 180.0))));
#endif
#ifdef NORTH_COLA
    this->super_points_.steer.v_limit = sqrt(this->super_points_.steer.p_steer_lat_acc * this->super_points_.steer.p_wheelbase / (tan(this->super_points_.steer.p_steer_amplify * fabs(this->super_points_.cur_steer.fabs_max * acos(-1.0) / 180.0))));
#endif
#ifdef TANK_6T
    this->super_points_.steer.v_limit = sqrt(this->super_points_.steer.p_steer_lat_acc * this->super_points_.cur_steer.fabs_max);
#endif

    for (auto & tmp_v: this->super_points_.v_constrained) {
        if (tmp_v > this->super_points_.steer.v_limit) {
            tmp_v = this->super_points_.steer.v_limit;
        }
    }
    this->super_points_.v_constrained[0] = this->super_points_.cur_speed.speed;
}

void CSMethod1::limit_by_ramp() {
    //// todo change to gpsdata
    if (this->super_points_.on_off.p_verbose) {
        ROS_INFO_STREAM("CS: limit_by_ramp");
    }

    //// todo  used to subs 'sensor_fusion_output' which is 10Hz
//    if (this->super_points_.msg_udt.addition.ramp) {
//        this->super_points_.ramp.update_count = 0;
//    } else {
//        ++this->super_points_.ramp.update_count;
//        if (this->super_points_.ramp.update_count >= 5) {
//            this->super_points_.ramp.update_count = 5;
//            return;
//        }
//    }

//    if (!this->super_points_.msg_udt.addition.ramp) {
//        ROS_ERROR_STREAM("CS: '/gpsdata' not update, ignore ramp");
//        return;
//    }

    //// todo  some thoughts about ramp
//    this->super_points_.ramp.slope = (tan(fabs(this->super_points_.ramp.pitch * acos(-1.0) / 180.0)) + tan(fabs(this->super_points_.ramp.roll * acos(-1.0) / 180.0)));
//    this->super_points_.ramp.slope_d = atan(this->super_points_.ramp.slope) / acos(-1.0) * 180.0;
//    this->super_points_.ramp.roll_ratio = tan(fabs(this->super_points_.ramp.roll * acos(-1.0) / 180.0)) / this->super_points_.ramp.slope;
//    if (this->super_points_.ramp.slope <= 0) {
//        this->super_points_.ramp.slope = 0;
//        this->super_points_.ramp.roll_ratio = 0;
//    }
//    if (this->super_points_.ramp.roll_ratio > 1) {
//        this->super_points_.ramp.roll_ratio = 1;
//    }

    //// todo pitch lvbo
#ifdef TOYOTA
    this->super_points_.ramp.pitch = this->gpswithHeading_.pitch + 2.009;
#endif
#ifdef NORTH_COLA
    this->super_points_.ramp.pitch = this->gpswithHeading_.pitch;
#endif

#ifdef TANK_6T
    this->super_points_.ramp.pitch = this->gpswithHeading_.pitch;
#endif

    this->super_points_.ramp.roll = this->gpswithHeading_.roll;

    //// todo  calculate v_ramp_limit
    //// v_limit = -1.5 * slope + 13
    //// v_limit = -1 * slope + 9
    this->super_points_.ramp.start_slope = 4;
    this->super_points_.ramp.v_limit = 20;
    double_t tmp_max_slope = std::max(fabs(this->super_points_.ramp.pitch), fabs(this->super_points_.ramp.roll));

    this->super_points_.ramp.slopes.erase(this->super_points_.ramp.slopes.begin());
    this->super_points_.ramp.slopes.emplace_back(tmp_max_slope);
    double_t tmp_slope_result = std::accumulate(this->super_points_.ramp.slopes.begin(), this->super_points_.ramp.slopes.end(), 0.0) / this->super_points_.ramp.slopes.size();

    if (tmp_slope_result < this->super_points_.ramp.start_slope) {return;}
    this->super_points_.ramp.v_limit = -1 * tmp_slope_result + 9;
    if (this->super_points_.ramp.v_limit < 3) {
        this->super_points_.ramp.v_limit = 3;
    }

    for (auto & tmp_v: this->super_points_.v_constrained) {
        if (tmp_v > this->super_points_.ramp.v_limit) {
            tmp_v = this->super_points_.ramp.v_limit;
        }
    }
    this->super_points_.v_constrained[0] = this->super_points_.cur_speed.speed;
    ROS_INFO_STREAM("CS: ramp limit speed: " << this->super_points_.ramp.v_limit);
}

void CSMethod1::limit_lat_acc() {
    this->super_points_.lat_acc.p_acc_lat_max = this->params_.acc_lat_max;

    if ((this->super_points_.running_mode == running_mode_type::dynamic_collision_ahead) || (this->super_points_.running_mode == running_mode_type::dynamic_collision_retreat)) {
        return;
    }

    if (this->super_points_.on_off.p_verbose) {
        ROS_INFO_STREAM("CS: limit_lat_acc");
    }

    this->super_points_.lat_acc.v_limit.clear();
    this->super_points_.lat_acc.v_limit = this->super_points_.v_constrained;

    for (count_type i = 1; i < this->super_points_.point_num; ++i) {
        if (this->super_points_.curv_zero_sign[i]) {
            this->super_points_.lat_acc.v_limit[i] = this->super_points_.v_constrained[i];
        } else {
            this->super_points_.lat_acc.v_limit[i] = std::min(sqrt(this->super_points_.lat_acc.p_acc_lat_max / fabs(this->super_points_.curv_final[i])), this->super_points_.v_constrained[i]);
        }
    }
    this->super_points_.lat_acc.v_limit[0] = this->super_points_.cur_speed.speed;

    for (count_type i = 1; i < this->super_points_.point_num; ++i) {
        if (this->super_points_.v_constrained[i] > this->super_points_.lat_acc.v_limit[i]) {
            this->super_points_.v_constrained[i] = this->super_points_.lat_acc.v_limit[i];
        }
    }
    this->super_points_.v_constrained[0] = this->super_points_.cur_speed.speed;
}

void CSMethod1::blind_handle() {
    this->super_points_.blind.p_blind_mode = this->params_.blind_mode;
    this->super_points_.blind.p_blind_slide_dec = this->params_.slide_dec + 0.01;
    this->super_points_.blind.p_blind_time = this->params_.blind_time;

    if (this->super_points_.on_off.p_verbose) {
        ROS_INFO_STREAM("CS: blind_handle");
    }

    if ((this->super_points_.cur_speed.speed + this->super_points_.blind.p_blind_slide_dec * this->super_points_.blind.p_blind_time) < 0) {
        this->super_points_.blind.p_blind_time = -this->super_points_.cur_speed.speed / this->super_points_.blind.p_blind_slide_dec;
    }

    this->super_points_.blind.end_index = this->super_points_.point_num - 1;
    this->super_points_.blind.area = this->super_points_.blind.p_blind_time * this->super_points_.cur_speed.speed + 0.5 * this->super_points_.blind.p_blind_slide_dec * pow(this->super_points_.blind.p_blind_time, 2.0);
    for (count_type i = 1; i < this->super_points_.point_num; ++i) {
        if (this->super_points_.s[i] >= this->super_points_.blind.area) {
            this->super_points_.blind.end_index = (((2.0 * this->super_points_.blind.area) > (this->super_points_.s[i] + this->super_points_.s[i - 1]))? i: (i - 1));
            break;
        }
    }
    if (this->super_points_.blind.end_index > this->super_points_.durex.safe_redundancy_index) {
        this->super_points_.blind.end_index = this->super_points_.durex.safe_redundancy_index;
    }

    if (this->super_points_.blind.end_index == 0) {
        this->super_points_.blind.p_blind_mode = this->params_.blind_mode_no;
    }

    if (this->super_points_.blind.p_blind_mode == this->params_.blind_mode_no) {
        return;
    }

    this->super_points_.blind.v_blind.clear();
    this->super_points_.blind.v_blind = this->super_points_.v_constrained;

    vel_type tmp_v;
    vel_type tmp_v_min = this->super_points_.cur_speed.speed;
    if (this->super_points_.blind.p_blind_mode == this->params_.blind_mode_local) {
        for (count_type i = 1; i <= this->super_points_.blind.end_index; ++i) {
            tmp_v = sqrt(pow(this->super_points_.cur_speed.speed, 2.0) + 2.0 * this->super_points_.blind.p_blind_slide_dec * this->super_points_.s[i]);
            if (this->super_points_.blind.v_blind[i] < tmp_v) {
                if (tmp_v_min > this->super_points_.blind.v_blind[i]) {
                    tmp_v_min = this->super_points_.blind.v_blind[i];
                }
                this->super_points_.blind.v_blind[i] = tmp_v;
            }
        }
    }

    if (this->super_points_.blind.p_blind_mode == this->params_.blind_mode_global) {
        bool tmp_marker = true;
        for (count_type i = 1; i <= this->super_points_.blind.end_index; ++i) {
            tmp_v = sqrt(pow(this->super_points_.cur_speed.speed, 2.0) + 2.0 * this->super_points_.blind.p_blind_slide_dec * this->super_points_.s[i]);
            if (this->super_points_.blind.v_blind[i] < tmp_v) {
                if (tmp_v_min > this->super_points_.blind.v_blind[i]) {
                    tmp_v_min = this->super_points_.blind.v_blind[i];
                }
                if (tmp_marker) {tmp_marker = false;}
            }
        }
        if (!tmp_marker) {
            for (count_type i = 1; i <= this->super_points_.blind.end_index; ++i) {
                this->super_points_.blind.v_blind[i] = sqrt(pow(this->super_points_.cur_speed.speed, 2.0) + 2.0 * this->super_points_.blind.p_blind_slide_dec * this->super_points_.s[i]);
            }
        }
    }

    //// todo, not correct here
//    if (this->super_points_.blind.p_blind_reactivate) {
//        if (this->super_points_.points[this->super_points_.blind.end_index].v.v_blind > tmp_v_min) {
//            this->super_points_.points[this->super_points_.blind.end_index].v.v_blind = tmp_v_min;
//        }
//    }

    for (count_type i = 1; i < this->super_points_.point_num; ++i) {
        if (this->super_points_.v_constrained[i] < this->super_points_.blind.v_blind[i]) {
            this->super_points_.v_constrained[i] = this->super_points_.blind.v_blind[i];
        }
    }
    this->super_points_.v_constrained[0] = this->super_points_.cur_speed.speed;
}

void CSMethod1::dynamic_collision() {
    //// todo params
    if (this->super_points_.on_off.p_verbose) {
        ROS_INFO_STREAM("CS: dynamic_collision");
    }

    if (this->super_points_.msg_udt.addition.collision) {
        this->super_points_.dynamic_collision.update_count = 0;
    } else {
        ++this->super_points_.dynamic_collision.update_count;
        if (this->super_points_.dynamic_collision.update_count >= 5) {
            this->super_points_.dynamic_collision.update_count = 5;
            return;
        }
    }

    this->super_points_.dynamic_collision.collision = (this->super_points_.dynamic_collision.update_count < 5) && (this->dymicol_point_.collision);

//
//    if (!this->super_points_.dynamic_collision.collision) {
//        return;
//    }
//
//    this->super_points_.dynamic_collision.v_limit.clear();
//    this->super_points_.dynamic_collision.v_limit = this->super_points_.v_constrained;
//
//    this->super_points_.dynamic_collision.real_distance = this->super_points_.dynamic_collision.distance - this->super_points_.dynamic_collision.p_safe_redundancy - this->super_points_.dynamic_collision.p_dec_delay * this->super_points_.cur_speed.speed;
//
//    for (count_type i = 0; i < this->super_points_.point_num; ++i) {
//        if (this->super_points_.s[i] >= this->super_points_.dynamic_collision.real_distance) {
//            if (i == 0) {i = 1;}
//            for (count_type j = i - 1; j <= this->super_points_.point_num; ++j) {
//                this->super_points_.dynamic_collision.v_limit[j] = 0;
//            }
//            break;
//        }
//    }
//
//    for (count_type i = 1; i < this->super_points_.point_num; ++i) {
//        if (this->super_points_.v_constrained[i] > this->super_points_.dynamic_collision.v_limit[i]) {
//            this->super_points_.v_constrained[i] = this->super_points_.dynamic_collision.v_limit[i];
//        }
//    }
//    this->super_points_.v_constrained[0] = this->super_points_.cur_speed.speed;
}

void CSMethod1::limit_lon_acc() {
    this->super_points_.lon_acc.p_acc_lon_max = this->params_.acc_lon_max;
    this->super_points_.lon_acc.p_jerk_N_A_cave = this->params_.jerk_N_A_cave;

#ifdef TOYOTA
    if (this->super_points_.set_v_max.p_v_max_general < 3) {
        this->super_points_.lon_acc.p_acc_lon_max = 0.25 * (this->super_points_.set_v_max.p_v_max_general + 1);
    } else {
        this->super_points_.lon_acc.p_acc_lon_max = 0.5 * (this->super_points_.set_v_max.p_v_max_general - 1);
    }

    if (this->super_points_.lon_acc.p_acc_lon_max < 0.5) {
        this->super_points_.lon_acc.p_acc_lon_max = 0.5;
    }
    if (this->super_points_.lon_acc.p_acc_lon_max > 2.5) {
        this->super_points_.lon_acc.p_acc_lon_max = 2.5;
    }
    if (this->super_points_.lon_acc.p_acc_lon_max > this->params_.acc_lon_max) {
        this->super_points_.lon_acc.p_acc_lon_max = this->params_.acc_lon_max;
    }

    if (this->super_points_.set_v_max.p_v_max_general < 2.1) {
        this->super_points_.lon_acc.p_jerk_N_A_cave = 0.01;
    }
#endif

#ifdef NORTH_COLA
    if (this->super_points_.set_v_max.p_v_max_general < 3) {
        this->super_points_.lon_acc.p_acc_lon_max = 0.25 * (this->super_points_.set_v_max.p_v_max_general + 1);
    } else {
        this->super_points_.lon_acc.p_acc_lon_max = 0.5 * (this->super_points_.set_v_max.p_v_max_general - 1);
    }

    if (this->super_points_.lon_acc.p_acc_lon_max < 0.5) {
        this->super_points_.lon_acc.p_acc_lon_max = 0.5;
    }
    if (this->super_points_.lon_acc.p_acc_lon_max > 2.5) {
        this->super_points_.lon_acc.p_acc_lon_max = 2.5;
    }
    if (this->super_points_.lon_acc.p_acc_lon_max > this->params_.acc_lon_max) {
        this->super_points_.lon_acc.p_acc_lon_max = this->params_.acc_lon_max;
    }

    if (this->super_points_.set_v_max.p_v_max_general < 2.1) {
        this->super_points_.lon_acc.p_jerk_N_A_cave = 0.01;
    }
#endif

    if (this->super_points_.on_off.p_verbose) {
        ROS_INFO_STREAM("CS: limit_lon_acc");
    }

    this->super_points_.lon_acc.v_limit.clear();
    this->super_points_.lon_acc.v_limit = this->super_points_.v_constrained;

    this->super_points_.tmp_acc.clear();
    this->super_points_.tmp_acc.assign(this->super_points_.point_num, 0);
    this->super_points_.tmp_acc[0] = this->super_points_.cur_acc.acc;

    vel_type v0; vel_type a0; vel_type s; vel_type jerk; vel_type v_up; vel_type v_down; vel_type a_up; vel_type a_down;
    for (count_type i = 1; i < this->super_points_.point_num; ++i) {
        v0 = this->super_points_.lon_acc.v_limit[i - 1];
        a0 = this->super_points_.tmp_acc[i - 1];
        if (a0 < 0) {a0 = 0;}
        s = this->super_points_.interval[i];
        jerk = this->super_points_.lon_acc.p_jerk_N_A_cave;
        v_up = this->super_points_.v_constrained[i];
        v_down = 0;
        a_up = this->super_points_.lon_acc.p_acc_lon_max;
        a_down = 0;
        this->super_points_.lon_acc.v_limit[i] = jerk_speed_solution(v0, a0, s, jerk, v_up, v_down, a_up, a_down);
        this->super_points_.tmp_acc[i] = (pow(this->super_points_.lon_acc.v_limit[i], 2.0) - pow(this->super_points_.lon_acc.v_limit[i - 1], 2.0)) * 0.5 / this->super_points_.interval[i];
    }

    for (count_type i = 1; i < this->super_points_.point_num; ++i) {
        if (this->super_points_.v_constrained[i] > this->super_points_.lon_acc.v_limit[i]) {
            this->super_points_.v_constrained[i] = this->super_points_.lon_acc.v_limit[i];
        }
    }
    this->super_points_.v_constrained[0] = this->super_points_.cur_speed.speed;
}

void CSMethod1::limit_slide_dec() {
    this->super_points_.slide_dec.p_slide_dec = this->params_.slide_dec + 0.01;
    this->super_points_.slide_dec.p_jerk_N_S_cave = this->params_.jerk_N_S_cave;
    this->super_points_.slide_dec.p_remove_dec_mode = this->params_.remove_dec_mode;
    this->super_points_.slide_dec.p_v_init_diff = this->params_.v_init_diff;

    this->super_points_.slide_dec.c_regular_min_speed =sqrt(std::min(this->params_.acc_lat_max, this->params_.steer_lat_acc) / this->params_.curvature_max);
    if (this->super_points_.slide_dec.c_regular_min_speed < 1) {
        this->super_points_.slide_dec.c_regular_min_speed = 1;
    }

    if (this->super_points_.on_off.p_verbose) {
        ROS_INFO_STREAM("CS: limit_slide_dec");
    }

    this->super_points_.slide_dec.v_limit.clear();
    this->super_points_.slide_dec.v_limit = this->super_points_.v_constrained;

    //// todo. modify if need
    this->super_points_.tmp_acc.clear();
    this->super_points_.tmp_acc.assign(this->super_points_.point_num, 0);

    this->super_points_.tmp_acc.back() = 0;
//    this->super_points_.points.back().tmp_acc = fabs(this->super_points_.constraints.slide_dec);

    vel_type v0; vel_type a0; vel_type s; vel_type jerk; vel_type v_up; vel_type v_down; vel_type a_up; vel_type a_down;
    for (count_type i = (this->super_points_.point_num - 2); i >= 0; --i) {
        v0 = this->super_points_.slide_dec.v_limit[i + 1];
        a0 = this->super_points_.tmp_acc[i + 1];
        if (a0 < 0) {a0 = 0;}
        s = this->super_points_.interval[i + 1];
        jerk = this->super_points_.slide_dec.p_jerk_N_S_cave;
        v_up = this->super_points_.v_constrained[i];
        v_down = 0;
        a_up = fabs(this->super_points_.slide_dec.p_slide_dec);
        if ((v0 >= this->super_points_.slide_dec.c_regular_min_speed) && (v0 >= this->super_points_.cur_speed.speed) && (this->super_points_.slide_dec.p_remove_dec_mode == this->params_.remove_dec_mode_fully)) {
            a_up = 0;
        }
        a_down = 0;
        this->super_points_.slide_dec.v_limit[i] = jerk_speed_solution(v0, a0, s, jerk, v_up, v_down, a_up, a_down);
        this->super_points_.tmp_acc[i] = (pow(this->super_points_.slide_dec.v_limit[i], 2.0) - pow(this->super_points_.slide_dec.v_limit[i + 1], 2.0)) * 0.5 / this->super_points_.interval[i + 1];
    }

    if (fabs(this->super_points_.slide_dec.v_limit[0] - this->super_points_.cur_speed.speed) > this->super_points_.slide_dec.p_v_init_diff) {
        this->super_points_.slide_dec.slide_gen_warn = true;
        ROS_WARN_STREAM("CS: slide_gen_warn");
        return;
    }

    for (count_type i = 1; i < this->super_points_.point_num; ++i) {
        if (this->super_points_.v_constrained[i] > this->super_points_.slide_dec.v_limit[i]) {
            this->super_points_.v_constrained[i] = this->super_points_.slide_dec.v_limit[i];
        }
    }
    this->super_points_.v_constrained[0] = this->super_points_.cur_speed.speed;
}

void CSMethod1::limit_lon_dec() {
    this->super_points_.lon_dec.p_slide_dec = this->params_.slide_dec + 0.01;
    this->super_points_.lon_dec.p_jerk_N_D_cave = this->params_.jerk_N_S_cave;
    this->super_points_.lon_dec.p_remove_dec_mode = this->params_.remove_dec_mode;
    this->super_points_.lon_dec.p_v_init_diff = this->params_.v_init_diff;
    this->super_points_.lon_dec.p_dec_lon_max = this->params_.dec_lon_max;

    this->super_points_.lon_dec.c_regular_min_speed =sqrt(std::min(this->params_.acc_lat_max, this->params_.steer_lat_acc) / this->params_.curvature_max);
    if (this->super_points_.lon_dec.c_regular_min_speed < 1) {
        this->super_points_.lon_dec.c_regular_min_speed = 1;
    }


    if (this->super_points_.on_off.p_verbose) {
        ROS_INFO_STREAM("CS: limit_lon_dec");
    }

    if (!this->super_points_.slide_dec.slide_gen_warn) {
        return;
    }

    this->super_points_.lon_dec.v_limit.clear();
    this->super_points_.lon_dec.v_limit = this->super_points_.v_constrained;

    //// todo. modify if need
    this->super_points_.tmp_acc.clear();
    this->super_points_.tmp_acc.assign(this->super_points_.point_num, 0);

    this->super_points_.tmp_acc.back() = 0;
//    this->super_points_.points.back().tmp_acc = fabs(this->super_points_.constraints.slide_dec);

    vel_type v0; vel_type a0; vel_type s; vel_type jerk; vel_type v_up; vel_type v_down; vel_type a_up; vel_type a_down;
    for (count_type i = (this->super_points_.point_num - 2); i >= 0; --i) {
        v0 = this->super_points_.lon_dec.v_limit[i + 1];
        a0 = this->super_points_.tmp_acc[i + 1];
        if (a0 < 0) {a0 = 0;}
        s = this->super_points_.interval[i + 1];
        jerk = this->super_points_.lon_dec.p_jerk_N_D_cave;
        v_up = this->super_points_.v_constrained[i];
        v_down = 0;
        a_up = fabs(this->super_points_.lon_dec.p_dec_lon_max);
        if (v0 >= this->super_points_.cur_speed.speed) {
            if (this->super_points_.lon_dec.p_remove_dec_mode == this->params_.remove_dec_mode_dec) {
                a_up = fabs(this->super_points_.lon_dec.p_slide_dec);
            }
            if (this->super_points_.lon_dec.p_remove_dec_mode == this->params_.remove_dec_mode_fully) {
                a_up = 0;
                if (v0 < this->super_points_.lon_dec.c_regular_min_speed) {
                    a_up = fabs(this->super_points_.lon_dec.p_slide_dec);
                }
            }
        }
        a_down = 0;
        this->super_points_.lon_dec.v_limit[i] = jerk_speed_solution(v0, a0, s, jerk, v_up, v_down, a_up, a_down);
        this->super_points_.tmp_acc[i] = (pow(this->super_points_.lon_dec.v_limit[i], 2.0) - pow(this->super_points_.lon_dec.v_limit[i + 1], 2.0)) * 0.5 / this->super_points_.interval[i + 1];
    }

    this->super_points_.lon_dec.dec_gen_warn = (fabs(this->super_points_.lon_dec.v_limit[0] - this->super_points_.cur_speed.speed) > this->super_points_.lon_dec.p_v_init_diff);

    if (this->super_points_.lon_dec.dec_gen_warn) {
        return;
    }

    for (count_type i = 1; i < this->super_points_.point_num; ++i) {
        if (this->super_points_.v_constrained[i] > this->super_points_.lon_dec.v_limit[i]) {
            this->super_points_.v_constrained[i] = this->super_points_.lon_dec.v_limit[i];
        }
    }
    this->super_points_.v_constrained[0] = this->super_points_.cur_speed.speed;
}

void CSMethod1::dec_fail_handle() {
    this->super_points_.dec_fail.p_dec_lon_max = this->params_.dec_lon_max;
    this->super_points_.dec_fail.p_remove_dec_mode = this->params_.remove_dec_mode;
    this->super_points_.dec_fail.p_slide_dec = this->params_.slide_dec + 0.01;
    this->super_points_.dec_fail.p_v_init_diff = this->params_.v_init_diff;
    this->super_points_.dec_fail.p_safe_dec = this->params_.safe_dec;

    this->super_points_.dec_fail.c_regular_min_speed =sqrt(std::min(this->params_.acc_lat_max, this->params_.steer_lat_acc) / this->params_.curvature_max);
    if (this->super_points_.dec_fail.c_regular_min_speed < 1) {
        this->super_points_.dec_fail.c_regular_min_speed = 1;
    }

    if (!this->super_points_.lon_dec.dec_gen_warn) {
        return;
    }

    if (this->super_points_.on_off.p_verbose) {
        ROS_INFO_STREAM("CS: dec_fail_handle");
    }

    this->super_points_.dec_fail.v_limit.clear();
    this->super_points_.dec_fail.v_limit = this->super_points_.v_constrained;

    //// todo. modify if need
    this->super_points_.tmp_acc.clear();
    this->super_points_.tmp_acc.assign(this->super_points_.point_num, 0);
    this->super_points_.tmp_acc.back() = 0;
//    this->super_points_.points.back().tmp_acc = fabs(this->super_points_.constraints.slide_dec);

    vel_type v0; vel_type a0; vel_type s; vel_type jerk; vel_type v_up; vel_type v_down; vel_type a_up; vel_type a_down;
    for (count_type i = (this->super_points_.point_num - 2); i >= 0; --i) {
        v0 = this->super_points_.dec_fail.v_limit[i + 1];
        a0 = this->super_points_.tmp_acc[i + 1];
        if (a0 < 0) {a0 = 0;}
        s = this->super_points_.interval[i + 1];
        jerk = 10000;
        v_up = this->super_points_.v_constrained[i];
        v_down = 0;
        a_up = fabs(this->super_points_.dec_fail.p_dec_lon_max);
        if (v0 >= this->super_points_.cur_speed.speed) {
            if (this->super_points_.dec_fail.p_remove_dec_mode == this->params_.remove_dec_mode_dec) {
                a_up = fabs(this->super_points_.dec_fail.p_slide_dec);
            }
            if (this->super_points_.dec_fail.p_remove_dec_mode == this->params_.remove_dec_mode_fully) {
                a_up = 0;
                if (v0 < this->super_points_.dec_fail.c_regular_min_speed) {
                    a_up = fabs(this->super_points_.dec_fail.p_slide_dec);
                }
            }
        }
        a_down = 0;
        this->super_points_.dec_fail.v_limit[i] = jerk_speed_solution(v0, a0, s, jerk, v_up, v_down, a_up, a_down);
        this->super_points_.tmp_acc[i] = (pow(this->super_points_.dec_fail.v_limit[i], 2.0) - pow(this->super_points_.dec_fail.v_limit[i + 1], 2.0)) * 0.5 / this->super_points_.interval[i + 1];
    }

    this->super_points_.dec_fail.dec_fail_gen_warn = (fabs(this->super_points_.dec_fail.v_limit[0] - this->super_points_.cur_speed.speed) > this->super_points_.dec_fail.p_v_init_diff);

    if (!this->super_points_.dec_fail.dec_fail_gen_warn) {
        for (count_type i = 1; i < this->super_points_.point_num; ++i) {
            if (this->super_points_.v_constrained[i] > this->super_points_.dec_fail.v_limit[i]) {
                this->super_points_.v_constrained[i] = this->super_points_.dec_fail.v_limit[i];
            }
        }
        this->super_points_.v_constrained[0] = this->super_points_.cur_speed.speed;
        return;
    }

    this->super_points_.dec_fail.v_limit.clear();
    this->super_points_.dec_fail.v_limit = this->super_points_.v_constrained;

    //// todo. modify if need
    this->super_points_.tmp_acc.clear();
    this->super_points_.tmp_acc.assign(this->super_points_.point_num, 0);
    this->super_points_.tmp_acc.back() = 0;
//    this->super_points_.points.back().tmp_acc = fabs(this->super_points_.constraints.slide_dec);

    for (count_type i = (this->super_points_.point_num - 2); i >= 0; --i) {
        v0 = this->super_points_.dec_fail.v_limit[i + 1];
        a0 = this->super_points_.tmp_acc[i + 1];
        if (a0 < 0) {a0 = 0;}
        s = this->super_points_.interval[i + 1];
        jerk = 10000;
        v_up = this->super_points_.v_constrained[i];
        v_down = 0;
        a_up = fabs(this->super_points_.dec_fail.p_safe_dec);
        if (v0 >= this->super_points_.cur_speed.speed) {
            if (this->super_points_.dec_fail.p_remove_dec_mode == this->params_.remove_dec_mode_dec) {
                a_up = fabs(this->super_points_.dec_fail.p_slide_dec);
            }
            if (this->super_points_.dec_fail.p_remove_dec_mode == this->params_.remove_dec_mode_fully) {
                a_up = 0;
                if (v0 < this->super_points_.dec_fail.c_regular_min_speed) {
                    a_up = fabs(this->super_points_.dec_fail.p_slide_dec);
                }
            }
        }
        a_down = 0;
        this->super_points_.dec_fail.v_limit[i] = jerk_speed_solution(v0, a0, s, jerk, v_up, v_down, a_up, a_down);
        this->super_points_.tmp_acc[i] = (pow(this->super_points_.dec_fail.v_limit[i], 2.0) - pow(this->super_points_.dec_fail.v_limit[i + 1], 2.0)) * 0.5 / this->super_points_.interval[i + 1];
    }

    this->super_points_.dec_fail.dec_fail_gen_warn2 = (fabs(this->super_points_.dec_fail.v_limit[0] - this->super_points_.cur_speed.speed) > this->super_points_.dec_fail.p_v_init_diff);

    for (count_type i = 1; i < this->super_points_.point_num; ++i) {
        if (this->super_points_.v_constrained[i] > this->super_points_.dec_fail.v_limit[i]) {
            this->super_points_.v_constrained[i] = this->super_points_.dec_fail.v_limit[i];
        }
    }
    this->super_points_.v_constrained[0] = this->super_points_.cur_speed.speed;
}

void CSMethod1::limit_jerk() {
    this->super_points_.jerk.p_jerk_N_A_vex = this->params_.jerk_N_A_vex;
    this->super_points_.jerk.p_jerk_N_D_vex = this->params_.jerk_N_D_vex;
    this->super_points_.jerk.p_jerk_N_S_vex = this->params_.jerk_N_S_vex;
    this->super_points_.jerk.p_jerk_cycle_diff = this->params_.jerk_cycle_diff;
    this->super_points_.jerk.p_jerk_cycle_times = this->params_.jerk_cycle_times;

#ifdef TOYOTA
    if (this->super_points_.set_v_max.p_v_max_general < 2.1) {
        this->super_points_.jerk.p_jerk_N_A_vex = -0.05;
    }
#endif
#ifdef NORTH_COLA
    if (this->super_points_.set_v_max.p_v_max_general < 2.1) {
        this->super_points_.jerk.p_jerk_N_A_vex = -0.05;
    }
#endif

    if (this->super_points_.on_off.p_verbose) {
        ROS_WARN_STREAM("CS: limit_jerk");
    }

    this->super_points_.jerk.v_limit.clear();
    this->super_points_.jerk.v_limit = this->super_points_.v_constrained;

    this->super_points_.tmp_v.clear();
    this->super_points_.tmp_v.assign(this->super_points_.point_num, 0);

    this->super_points_.tmp_v[0] = this->super_points_.jerk.v_limit[0];
    this->super_points_.tmp_v.back() = this->super_points_.jerk.v_limit.back();
    uint16_t tmp_cycle_times = 0;
    vel_type tmp_diff1 = 0;
    vel_type tmp_diff2 = 0;
    do {
        ++tmp_cycle_times;
        vel_type v_1; vel_type v1; vel_type s0; vel_type s1; vel_type jerk1;
        vel_type tmp_v;
        for (count_type i = 1; i < this->super_points_.point_num - 1; ++i) {
            this->super_points_.tmp_v[i] = this->super_points_.jerk.v_limit[i];
            v_1 = this->super_points_.jerk.v_limit[i - 1];
            v1 = this->super_points_.jerk.v_limit[i + 1];
            s0 = this->super_points_.interval[i];
            s1 = this->super_points_.interval[i + 1];
            if (v_1 <= v1) {
                jerk1 = this->super_points_.jerk.p_jerk_N_A_vex;
            } else {
                if (this->super_points_.slide_dec.slide_gen_warn) {
                    jerk1 = this->super_points_.jerk.p_jerk_N_D_vex;
                } else {
                    jerk1 = this->super_points_.jerk.p_jerk_N_S_vex;
                }
            }
            tmp_v = jerk_speed_solution(v_1, v1, s0, s1, jerk1);
            if (this->super_points_.jerk.v_limit[i] > tmp_v) {
                this->super_points_.jerk.v_limit[i] = tmp_v;
            }
            tmp_diff1 = fabs(this->super_points_.jerk.v_limit[i] - this->super_points_.tmp_v[i]);
            if (tmp_diff2 < tmp_diff1) {
                tmp_diff2 = tmp_diff1;
            }
        }
    } while ((tmp_diff2 > this->super_points_.jerk.p_jerk_cycle_diff) && (tmp_cycle_times < this->super_points_.jerk.p_jerk_cycle_times));

    for (count_type i = 1; i < this->super_points_.point_num; ++i) {
        if (this->super_points_.v_constrained[i] > this->super_points_.jerk.v_limit[i]) {
            this->super_points_.v_constrained[i] = this->super_points_.jerk.v_limit[i];
        }
    }
    this->super_points_.v_constrained[0] = this->super_points_.cur_speed.speed;
}

void CSMethod1::cal_acc() {
    if (this->super_points_.on_off.p_verbose) {
        ROS_WARN_STREAM("CS: cal_acc");
    }

    this->super_points_.acc.clear();
    this->super_points_.acc.assign(this->super_points_.point_num, 0);

    this->super_points_.v_constrained[0] = this->super_points_.cur_speed.speed;
    this->super_points_.acc[0] = this->super_points_.cur_acc.acc;
    for (count_type i = 1; i < this->super_points_.point_num; ++i) {
        this->super_points_.acc[i] = (pow(this->super_points_.v_constrained[i], 2.0) - pow(this->super_points_.v_constrained[i - 1], 2.0)) * 0.5 / this->super_points_.interval[i];
        if (this->super_points_.acc[i] > this->params_.acc_lon_max) {
            this->super_points_.acc[i] = this->params_.acc_lon_max;
        }
        if (this->super_points_.acc[i] < this->params_.safe_dec) {
            this->super_points_.acc[i] = this->params_.safe_dec;
        }
    }
}

void CSMethod1::cal_time() {
    if (this->super_points_.on_off.p_verbose) {
        ROS_WARN_STREAM("CS: cal_time");
    }

    this->super_points_.time.time.clear();
    this->super_points_.time.interval.clear();
    this->super_points_.time.time.assign(this->super_points_.point_num, 0);
    this->super_points_.time.interval.assign(this->super_points_.point_num, 0);

    this->super_points_.time.time[0] = 0;
    this->super_points_.time.interval[0] = 0;
    for (count_type i = 1; i < this->super_points_.point_num; ++i) {
        this->super_points_.time.interval[i] = 2.0 * this->super_points_.interval[i] / (this->super_points_.v_constrained[i - 1] + this->super_points_.v_constrained[i]);
        this->super_points_.time.time[i] = this->super_points_.time.time[i - 1] + this->super_points_.time.interval[i];
    }
}

void CSMethod1::issue_cal() {
    this->super_points_.issue.p_acc_delay = this->params_.acc_delay;
    this->super_points_.issue.p_dec_delay = this->params_.dec_delay;
    this->super_points_.issue.p_min_brake_dec = this->params_.min_brake_dec;
    this->super_points_.issue.p_acc_lon_max = this->params_.acc_lon_max;
    this->super_points_.issue.p_safe_dec = this->params_.safe_dec;
    this->super_points_.issue.p_not_pub_dead_zone = this->params_.not_pub_dead_zone;

    if (this->super_points_.on_off.p_verbose) {
        ROS_WARN_STREAM("CS: issue_cal");
    }

    bool tmp_find = false;
    for (count_type i = 0; i < this->super_points_.point_num; ++i) {
        if (this->super_points_.time.time[i] >= this->super_points_.issue.p_dec_delay) {
            tmp_find = true;
            this->super_points_.issue.delay_index = i - 1;
            this->super_points_.issue.ratio = (this->super_points_.issue.p_dec_delay - this->super_points_.time.time[i - 1]) / this->super_points_.time.interval[i];
            break;
        }
    }

    if (!tmp_find) {
        this->super_points_.issue.delay_index = this->super_points_.point_num - 2;
        this->super_points_.issue.ratio = 1.0;
    }

    this->super_points_.issue.v = this->super_points_.v_constrained[this->super_points_.issue.delay_index] + this->super_points_.issue.ratio * (this->super_points_.v_constrained[this->super_points_.issue.delay_index + 1] - this->super_points_.v_constrained[this->super_points_.issue.delay_index]);

    if (this->super_points_.issue.v < this->super_points_.cur_speed.speed) {
        vel_type tmp_min_acc = this->super_points_.acc[1];
        for (count_type i = 1; i <= this->super_points_.issue.delay_index + 1; ++i) {
            if (tmp_min_acc > this->super_points_.acc[i]) {
                tmp_min_acc = this->super_points_.acc[i];
            }
        }
        this->super_points_.issue.acc = tmp_min_acc;
    } else {
        for (count_type i = 0; i < this->super_points_.point_num; ++i) {
            if (this->super_points_.time.time[i] >= this->super_points_.issue.p_acc_delay) {
                this->super_points_.issue.delay_index = i - 1;
                this->super_points_.issue.ratio = (this->super_points_.issue.p_acc_delay - this->super_points_.time.time[i - 1]) / this->super_points_.time.interval[i];
                break;
            }
        }
        this->super_points_.issue.v = this->super_points_.v_constrained[this->super_points_.issue.delay_index] + this->super_points_.issue.ratio * (this->super_points_.v_constrained[this->super_points_.issue.delay_index + 1] - this->super_points_.v_constrained[this->super_points_.issue.delay_index]);
        this->super_points_.issue.acc = this->super_points_.acc[this->super_points_.issue.delay_index] + this->super_points_.issue.ratio * (this->super_points_.acc[this->super_points_.issue.delay_index + 1] - this->super_points_.acc[this->super_points_.issue.delay_index]);
        if (this->super_points_.issue.acc < 0.03) {
            this->super_points_.issue.acc = 0.03;
        }
    }

    if (this->super_points_.issue.p_not_pub_dead_zone) {
        if ((this->super_points_.issue.acc < this->params_.slide_dec) && (this->super_points_.issue.acc > this->super_points_.issue.p_min_brake_dec)) {
            this->super_points_.issue.acc = this->super_points_.issue.p_min_brake_dec;
        }
    }

    if (this->super_points_.dec_fail.dec_fail_gen_warn2) {
        this->super_points_.issue.acc = this->super_points_.issue.p_safe_dec;
    }

    if (this->super_points_.issue.acc > this->super_points_.issue.p_acc_lon_max) {
        this->super_points_.issue.acc = this->super_points_.issue.p_acc_lon_max;
    }
    if (this->super_points_.issue.acc < this->super_points_.issue.p_safe_dec) {
        this->super_points_.issue.acc = this->super_points_.issue.p_safe_dec;
    }

    this->super_points_.issue.direction = this->super_points_.direction;

#ifdef TANK_6T
    if (this->super_points_.issue.v < 0.1) {
        this->super_points_.issue.v = 0.1;
    }

    if (((this->super_points_.issue.v - this->super_points_.cur_speed.speed) > 0.075) && ((this->super_points_.issue.v - this->super_points_.cur_speed.speed) < 0.3)) {
        this->super_points_.issue.v = this->super_points_.cur_speed.speed + 0.3;
    }

//    if (((this->super_points_.cur_speed.speed - this->super_points_.issue.v) > 0.075) && ((this->super_points_.cur_speed.speed - this->super_points_.issue.v) < 0.3)) {
//        this->super_points_.issue.v = this->super_points_.cur_speed.speed - 0.3;
//    }

    if (this->super_points_.issue.direction == gear_type::R) {
        this->super_points_.issue.v = -this->super_points_.issue.v;
    }

//    if (this->super_points_.issue.direction == gear_type::D1) {
//        this->super_points_.issue.acc = this->super_points_.issue.v;
//    }
//    if (this->super_points_.issue.direction == gear_type::R) {
//        this->super_points_.issue.acc = -this->super_points_.issue.v;
//    }
#endif
}

void CSMethod1::brake_amount() {
    this->super_points_.brake_amount.p_brake_amount_mode = this->params_.brake_amount_mode;
    this->super_points_.brake_amount.p_slide_dec = this->params_.slide_dec + 0.01;
    this->super_points_.brake_amount.p_acc_time_limit = this->params_.acc_time_limit;
    this->super_points_.brake_amount.p_brake_amount_amplify = this->params_.brake_amount_amplify;
    this->super_points_.brake_amount.p_brake_amount_multiple_coefficient = this->params_.brake_amount_multiple_coefficient;
    this->super_points_.brake_amount.p_brake_amplify_revise = this->params_.brake_amplify_revise;
    this->super_points_.brake_amount.p_brake_distinguish = this->params_.brake_distinguish;

    if (this->super_points_.on_off.p_verbose) {
        ROS_WARN_STREAM("CS: brake_amount");
    }

    if ((this->super_points_.cur_speed.speed < 0.1) || (this->super_points_.issue.v < 0.1)) {
        this->super_points_.brake_amount.amount = 0;
        this->super_points_.brake_amount.decs.clear();
        this->super_points_.brake_amount.dec_times.clear();
        this->super_points_.brake_amount.acc_times.clear();
        ROS_WARN_STREAM("CS: brake amount returned");
        return;
    }

//    if (this->super_points_.cur_speed.speed < 0.1) {return;}
//    if (this->super_points_.issue.v < 0.5) {return;}

    if (this->super_points_.issue.acc >= this->super_points_.brake_amount.p_brake_distinguish) {
        this->super_points_.brake_amount.dec_times.clear();
        this->super_points_.brake_amount.decs.clear();

        if (this->super_points_.brake_amount.acc_times.empty()) {
            this->super_points_.brake_amount.acc_times.assign(2, ros::Time::now().toSec());
        } else {
            this->super_points_.brake_amount.acc_times[1] = ros::Time::now().toSec();
        }
        this->super_points_.brake_amount.acc_time = this->super_points_.brake_amount.acc_times.back() - this->super_points_.brake_amount.acc_times[0];
        if (this->super_points_.brake_amount.acc_time >= this->super_points_.brake_amount.p_acc_time_limit) {
            this->super_points_.brake_amount.amount = 0;
            this->super_points_.brake_amount.dec_times.clear();
            this->super_points_.brake_amount.decs.clear();
        }
        return;
    } else {
        this->super_points_.brake_amount.acc_times.clear();

        this->super_points_.brake_amount.dec_times.emplace_back(ros::Time::now().toSec());
        this->super_points_.brake_amount.decs.emplace_back(this->super_points_.issue.acc);
        if (this->super_points_.brake_amount.dec_times.size() == 1) {
            this->super_points_.brake_amount.dec_start_v = this->super_points_.cur_speed.speed;
        }
        if (this->super_points_.brake_amount.dec_times.size() > 1) {
            if (this->super_points_.brake_amount.dec_times.size() > 2) {
                this->super_points_.brake_amount.dec_times.erase(this->super_points_.brake_amount.dec_times.begin());
                this->super_points_.brake_amount.decs.erase(this->super_points_.brake_amount.decs.begin());
            }
            if (this->super_points_.brake_amount.p_brake_amplify_revise) {
                double_t tmp_amplify = (this->super_points_.brake_amount.dec_start_v - this->super_points_.cur_speed.speed) / this->super_points_.brake_amount.amount;
                if (this->super_points_.brake_amount.p_brake_amount_amplify < tmp_amplify) {
                    this->super_points_.brake_amount.p_brake_amount_amplify = tmp_amplify;
                }
            }

//            double_t tmp_amount = this->super_points_.brake_amount.p_brake_amount_amplify * fabs(0.5 * (this->super_points_.brake_amount.decs[0] + this->super_points_.brake_amount.decs[1]) * (this->super_points_.brake_amount.dec_times[1] - this->super_points_.brake_amount.dec_times[0]));
//            this->super_points_.brake_amount.amount += this->super_points_.brake_amount.p_brake_amount_amplify * fabs(0.5 * (this->super_points_.brake_amount.decs[0] + this->super_points_.brake_amount.decs[1]) * (this->super_points_.brake_amount.dec_times[1] - this->super_points_.brake_amount.dec_times[0]));
            if ((this->super_points_.brake_amount.dec_start_v - this->super_points_.brake_amount.amount) <= this->super_points_.issue.v) {
                //// todo, modify if need
                if (this->super_points_.brake_amount.p_brake_amount_mode == this->params_.brake_amount_mode_zero) {
                    this->super_points_.issue.acc = 0;
                }
                if (this->super_points_.brake_amount.p_brake_amount_mode == this->params_.brake_amount_mode_slide) {
                    this->super_points_.issue.acc = this->super_points_.brake_amount.p_slide_dec;
                }
                if (this->super_points_.brake_amount.p_brake_amount_mode == this->params_.brake_amount_mode_multiple) {
                    this->super_points_.issue.acc *= this->super_points_.brake_amount.p_brake_amount_multiple_coefficient;
                }
            } else {
                this->super_points_.brake_amount.amount += this->super_points_.brake_amount.p_brake_amount_amplify * fabs(0.5 * (this->super_points_.brake_amount.decs[0] + this->super_points_.brake_amount.decs[1]) * (this->super_points_.brake_amount.dec_times[1] - this->super_points_.brake_amount.dec_times[0]));
            }
        }
    }
}

vel_type CSMethod1::jerk_speed_solution(const vel_type v0, const vel_type a0, const vel_type s, const vel_type jerk, const vel_type v_up, const vel_type v_down, const vel_type a_up, const vel_type a_down) {
    vel_type v1, a1;
    a1 = a0 + jerk * s / v0;
    if (v0 < 0.01) {
        a1 = a0 + jerk * s / 0.01;
    }
    if (a1 > a_up) {a1 = a_up;}
    if (a1 < a_down) {a1 = a_down;}
    v1 = sqrt(pow(v0, 2.0) + 2.0 * a1 * s);
    if (v1 > v_up) {v1 = v_up;}
    if (v1 < v_down) {v1 = v_down;}
    return v1;
}

vel_type CSMethod1::jerk_speed_solution(const vel_type v_1, const vel_type v1, const vel_type s0, const vel_type s1, const vel_type jerk) {
    vel_type v0;
    v0 = sqrt((s0 * pow(v1, 2.0) + s1 * pow(v_1, 2.0) - jerk * 4.0 * s0 * pow(s1, 2.0) / (v_1 + v1)) / (s0 + s1));
    return v0;
}

void CSMethod1::debug_handle() {
    this->super_points_.on_off.p_enable_debug = this->params_.enable_debug;

    if (!this->super_points_.on_off.p_enable_debug) {
        return;
    }

    this->speed_debug_.points.clear();
    for (count_type i = 0; i < this->super_points_.point_num; ++i) {
        this->speed_debug_point_.v.v_constrained = this->super_points_.v_constrained[i];
        this->speed_debug_point_.curv.curv_final = this->super_points_.curv_final[i];
        this->speed_debug_point_.time.time = this->super_points_.time.time[i];
        this->speed_debug_point_.x = this->super_points_.x_points[i];
        this->speed_debug_point_.y = this->super_points_.y_points[i];
        this->speed_debug_point_.s = this->super_points_.s[i];
        this->speed_debug_point_.acc = this->super_points_.acc[i];

        this->speed_debug_.points.emplace_back(this->speed_debug_point_);
    }
    this->speed_debug_.pub_ros_time = this->super_points_.time_check.pub_ros_time / 1000;
    this->speed_debug_.cur_steer.steer = this->super_points_.cur_steer.steer;
    this->speed_debug_.issue.v = this->super_points_.issue.v;
    this->speed_debug_.issue.acc = this->super_points_.issue.acc;
    this->debug_pub_.publish(speed_debug_);
}

void CSMethod1::reconfigureRequest(cs_method1Config & config, uint32_t level) {
    this->super_points_.on_off.p_program_halt = config.halt1 && config.halt2;
    if (config.params_lock) {
        ROS_WARN_STREAM("CS: params is locked, not set");
        return;
    }
    this->params_.fromConfig(config);
    this->super_points_.on_off.p_issue_mode = this->params_.issue_mode;
    this->super_points_.on_off.p_verbose = this->params_.verbose;
    ROS_WARN_STREAM("CS: params is unlocked, set to params");
}

void CSMethod1::fake_state() {
    this->super_points_.fake.p_fake = this->params_.fake;
    if (!this->super_points_.fake.p_fake) {
        return;
    }
    this->super_points_.fake.p_speed_fake = this->params_.speed_fake;
    this->super_points_.fake.p_acc_fake = this->params_.acc_fake;
    this->super_points_.fake.p_steer_fake = this->params_.steer_fake;
    this->super_points_.fake.p_fake_speed = this->params_.fake_speed;
    this->super_points_.fake.p_fake_acc = this->params_.fake_acc;
    this->super_points_.fake.p_fake_steer = this->params_.fake_steer;
    if (this->super_points_.fake.p_speed_fake) {
        this->super_points_.cur_speed.speed = this->super_points_.fake.p_fake_speed;
    }
    if (this->super_points_.fake.p_acc_fake) {
        this->super_points_.cur_acc.acc = this->super_points_.fake.p_fake_acc;
    }
    if (this->super_points_.fake.p_steer_fake) {
        this->super_points_.cur_steer.fabs_max = this->super_points_.fake.p_fake_steer;
    }
}

void CSMethod1::fake_issue() {
    this->super_points_.fake.p_fake = this->params_.fake;
    if (!this->super_points_.fake.p_fake) {
        return;
    }
    this->super_points_.fake.p_issue_acc_fake = this->params_.issue_acc_fake;
    this->super_points_.fake.p_fake_issue_acc = this->params_.fake_issue_acc;
    this->super_points_.fake.p_issue_direction_fake = this->params_.issue_direction_fake;
    switch (this->params_.fake_issue_direction) {
        case this->params_.fake_issue_direction_N: {
            this->super_points_.fake.p_fake_issue_direction = gear_type::N;
            break;
        }
        case this->params_.fake_issue_direction_D1: {
            this->super_points_.fake.p_fake_issue_direction = gear_type::D1;
            break;
        }
        case this->params_.fake_issue_direction_R: {
            this->super_points_.fake.p_fake_issue_direction = gear_type::R;
            break;
        }
    }

    if (this->super_points_.fake.p_issue_acc_fake) {
        this->super_points_.issue.acc = this->super_points_.fake.p_fake_issue_acc;
    }
    if (this->super_points_.fake.p_issue_direction_fake) {
        this->super_points_.issue.direction = this->super_points_.fake.p_fake_issue_direction;
    }
}

void CSMethod1::publish() {
    if (this->super_points_.on_off.p_verbose) {
        ROS_WARN_STREAM("CS: publish");
    }

    /// **************************************************************
    /// speed ctrl publish

    ++this->super_points_.cycle_count;
    this->speed_ctrl_.count = this->super_points_.cycle_count;
    this->speed_ctrl_.cur_speed = this->super_points_.cur_speed.speed;
    this->speed_ctrl_.cur_acc = this->super_points_.cur_acc.acc;

    this->speed_ctrl_.issue_v = this->super_points_.issue.v;
    this->speed_ctrl_.issue_acc = this->super_points_.issue.acc;

    switch (this->super_points_.issue.direction) {
        case gear_type::N: {
            this->speed_ctrl_.direction = 1;
        }
        case gear_type::D1: {
            this->speed_ctrl_.direction = 2;
            break;
        }
        case gear_type::R: {
            this->speed_ctrl_.direction = 3;
            break;
        }
        default: {
            this->speed_ctrl_.direction = 1;
            break;
        }
    }

    this->super_points_.time_check.pub_ros_time = ros::Time::now().toSec() * 1000;

    this->speed_ctrl_.rc_path_time = this->super_points_.time_check.rc_ros_time;
    this->speed_ctrl_.pub_acc_time = this->super_points_.time_check.pub_ros_time;

    ////  debug
    this->speed_ctrl_.total_s = this->super_points_.s.back();
    this->speed_ctrl_.curv_max = this->pointsAttributes_.curv_cal.curvature_max;
    this->speed_ctrl_.running_mode = uint8_t(this->super_points_.running_mode);
    ////  debug end

    this->ctrl_pub_.publish(this->speed_ctrl_);
    /// ***********************************************************************

    /// ************************************************************************
    /// trajectory publish
    for (count_type i = 0; i < this->super_points_.point_num; ++i) {
        this->trajectory_.points[i].velocity.linear.x = this->super_points_.v_constrained[i];
    }

#ifdef TOYOTA
    this->trajectory_.issued_acc = this->super_points_.issue.acc;
#endif
#ifdef NORTH_COLA
    this->trajectory_.issued_acc = this->super_points_.issue.acc;
#endif

#ifdef TANK_6T
    this->trajectory_.issued_acc = this->super_points_.issue.v;
#endif

    this->traj_pub_.publish(this->trajectory_);
    /// ************************************************************************

    ROS_INFO_STREAM("CS: count: " << this->super_points_.cycle_count << "  issue_v: " << this->super_points_.issue.v << "  issue_acc: " << this->super_points_.issue.acc << "  shift: " << this->gear_strings_[this->super_points_.issue.direction]);
}

void CSMethod1::cal_assist_params() {
    this->super_points_.assist_params.smart_safe_distance = 0;

#ifdef TOYOTA
    double_t general_space_front = 2.5;
    double_t general_space_rear = 2.5;
    double_t search_space_front = 3.8;
    double_t search_space_rear = 1.2;

    /// general_ahead
    if (this->super_points_.running_mode == running_mode_type::general_ahead) {
        this->super_points_.assist_params.smart_safe_distance = general_space_front + 1;
    }

    /// general_retreat
    if (this->super_points_.running_mode == running_mode_type::general_retreat) {
        this->super_points_.assist_params.smart_safe_distance = 0;
    }

    /// dynamic_collision_ahead
    if (this->super_points_.running_mode == running_mode_type::dynamic_collision_ahead) {
        this->super_points_.assist_params.smart_safe_distance = general_space_front + 1;
    }

    /// dynamic_collision_retreat
    if (this->super_points_.running_mode == running_mode_type::dynamic_collision_retreat) {
        this->super_points_.assist_params.smart_safe_distance = 0;
    }

    /// foggy_ahead
    if (this->super_points_.running_mode == running_mode_type::foggy_ahead) {
        this->super_points_.assist_params.smart_safe_distance = general_space_front + 1;
    }

    /// foggy_retreat
    if (this->super_points_.running_mode == running_mode_type::foggy_retreat) {
        this->super_points_.assist_params.smart_safe_distance = 0;
    }

    /// inspect_ahead
    if (this->super_points_.running_mode == running_mode_type::inspect_ahead) {
        this->super_points_.assist_params.smart_safe_distance = general_space_front + 1;
    }

    /// inspect_retreat
    if (this->super_points_.running_mode == running_mode_type::inspect_retreat) {
        this->super_points_.assist_params.smart_safe_distance = 0;
    }

    /// search_ahead
    if (this->super_points_.running_mode == running_mode_type::search_ahead) {
        this->super_points_.assist_params.smart_safe_distance = 0;
    }

    /// search_retreat
    if (this->super_points_.running_mode == running_mode_type::search_retreat) {
        this->super_points_.assist_params.smart_safe_distance = 0;
    }

    /// search_direction_diff
    if (this->super_points_.running_mode == running_mode_type::search_direction_diff) {
        this->super_points_.assist_params.smart_safe_distance = 0;
    }
#endif

#ifdef NORTH_COLA
    double_t general_space_front = 2.5;
    double_t general_space_rear = 2.5;
    double_t search_space_front = 3.8;
    double_t search_space_rear = 1.2;

    /// general_ahead
    if (this->super_points_.running_mode == running_mode_type::general_ahead) {
        this->super_points_.assist_params.smart_safe_distance = general_space_front + 1;
    }

    /// general_retreat
    if (this->super_points_.running_mode == running_mode_type::general_retreat) {
        this->super_points_.assist_params.smart_safe_distance = 0;
    }

    /// dynamic_collision_ahead
    if (this->super_points_.running_mode == running_mode_type::dynamic_collision_ahead) {
        this->super_points_.assist_params.smart_safe_distance = general_space_front + 1;
    }

    /// dynamic_collision_retreat
    if (this->super_points_.running_mode == running_mode_type::dynamic_collision_retreat) {
        this->super_points_.assist_params.smart_safe_distance = 0;
    }

    /// foggy_ahead
    if (this->super_points_.running_mode == running_mode_type::foggy_ahead) {
        this->super_points_.assist_params.smart_safe_distance = general_space_front + 1;
    }

    /// foggy_retreat
    if (this->super_points_.running_mode == running_mode_type::foggy_retreat) {
        this->super_points_.assist_params.smart_safe_distance = 0;
    }

    /// inspect_ahead
    if (this->super_points_.running_mode == running_mode_type::inspect_ahead) {
        this->super_points_.assist_params.smart_safe_distance = general_space_front + 1;
    }

    /// inspect_retreat
    if (this->super_points_.running_mode == running_mode_type::inspect_retreat) {
        this->super_points_.assist_params.smart_safe_distance = 0;
    }

    /// search_ahead
    if (this->super_points_.running_mode == running_mode_type::search_ahead) {
        this->super_points_.assist_params.smart_safe_distance = 0;
    }

    /// search_retreat
    if (this->super_points_.running_mode == running_mode_type::search_retreat) {
        this->super_points_.assist_params.smart_safe_distance = 0;
    }

    /// search_direction_diff
    if (this->super_points_.running_mode == running_mode_type::search_direction_diff) {
        this->super_points_.assist_params.smart_safe_distance = 0;
    }
#endif

#ifdef TANK_6T
    double_t general_space_front = 2.5;
    double_t general_space_rear = 2.5;
    double_t search_space_front = 3.8;
    double_t search_space_rear = 1.2;

    /// general_ahead
    if (this->super_points_.running_mode == running_mode_type::general_ahead) {
        this->super_points_.assist_params.smart_safe_distance = general_space_front + 1;
    }

    /// general_retreat
    if (this->super_points_.running_mode == running_mode_type::general_retreat) {
        this->super_points_.assist_params.smart_safe_distance = 0;
    }

    /// dynamic_collision_ahead
    if (this->super_points_.running_mode == running_mode_type::dynamic_collision_ahead) {
        this->super_points_.assist_params.smart_safe_distance = general_space_front + 1;
    }

    /// dynamic_collision_retreat
    if (this->super_points_.running_mode == running_mode_type::dynamic_collision_retreat) {
        this->super_points_.assist_params.smart_safe_distance = 0;
    }

    /// foggy_ahead
    if (this->super_points_.running_mode == running_mode_type::foggy_ahead) {
        this->super_points_.assist_params.smart_safe_distance = general_space_front + 1;
    }

    /// foggy_retreat
    if (this->super_points_.running_mode == running_mode_type::foggy_retreat) {
        this->super_points_.assist_params.smart_safe_distance = 0;
    }

    /// inspect_ahead
    if (this->super_points_.running_mode == running_mode_type::inspect_ahead) {
        this->super_points_.assist_params.smart_safe_distance = general_space_front + 1;
    }

    /// inspect_retreat
    if (this->super_points_.running_mode == running_mode_type::inspect_retreat) {
        this->super_points_.assist_params.smart_safe_distance = 0;
    }

    /// search_ahead
    if (this->super_points_.running_mode == running_mode_type::search_ahead) {
        this->super_points_.assist_params.smart_safe_distance = 1;
    }

    /// search_retreat
    if (this->super_points_.running_mode == running_mode_type::search_retreat) {
        this->super_points_.assist_params.smart_safe_distance = 1;
    }

    /// search_direction_diff
    if (this->super_points_.running_mode == running_mode_type::search_direction_diff) {
        this->super_points_.assist_params.smart_safe_distance = 0;
    }
#endif
}

#ifdef TOYOTA
void CSMethod1::speedCb(const control_msgs::GetECUReport msg) {
    if (this->super_points_.on_off.p_verbose) {
        ROS_INFO_STREAM("CS: speedCb");
    }
    this->getECUReport_ = msg;
    this->super_points_.cur_speed.speed = this->getECUReport_.speed.velocity.linear.x;
    this->super_points_.cur_speed.seq.erase(this->super_points_.cur_speed.seq.begin());
    this->super_points_.cur_speed.seq.emplace_back(this->super_points_.cur_speed.speed);

    this->super_points_.cur_acc.acc = this->super_points_.cur_speed.seq.back() - this->super_points_.cur_speed.seq[0];

    this->super_points_.cur_steer.steer = this->getECUReport_.steer_cur.steer;
    this->super_points_.cur_steer.seq.erase(this->super_points_.cur_steer.seq.begin());
    this->super_points_.cur_steer.seq.emplace_back(this->super_points_.cur_steer.steer);
    this->super_points_.cur_steer.fabs_max = fabs(this->super_points_.cur_steer.steer);
    for (const auto & tmp_steer : this->super_points_.cur_steer.seq) {
        if (this->super_points_.cur_steer.fabs_max < fabs(tmp_steer)) {
            this->super_points_.cur_steer.fabs_max = fabs(tmp_steer);
        }
    }

    this->super_points_.msg_udt.update.items.speed = true;
}
#endif

#ifdef NORTH_COLA
    void CSMethod1::speedCb(const control_msgs::GetECUReport msg) {
    if (this->super_points_.on_off.p_verbose) {
        ROS_INFO_STREAM("CS: speedCb");
    }
    this->getECUReport_ = msg;
    this->super_points_.cur_speed.speed = this->getECUReport_.speed.velocity.linear.x;
    this->super_points_.cur_speed.seq.erase(this->super_points_.cur_speed.seq.begin());
    this->super_points_.cur_speed.seq.emplace_back(this->super_points_.cur_speed.speed);

    this->super_points_.cur_acc.acc = this->super_points_.cur_speed.seq.back() - this->super_points_.cur_speed.seq[0];

    this->super_points_.cur_steer.steer = this->getECUReport_.steer_cur.steer;
    this->super_points_.cur_steer.seq.erase(this->super_points_.cur_steer.seq.begin());
    this->super_points_.cur_steer.seq.emplace_back(this->super_points_.cur_steer.steer);
    this->super_points_.cur_steer.fabs_max = fabs(this->super_points_.cur_steer.steer);
    for (const auto & tmp_steer : this->super_points_.cur_steer.seq) {
        if (this->super_points_.cur_steer.fabs_max < fabs(tmp_steer)) {
            this->super_points_.cur_steer.fabs_max = fabs(tmp_steer);
        }
    }

    this->super_points_.msg_udt.update.items.speed = true;
}
#endif

#ifdef TANK_6T
//  * 3.14 / 1650
void CSMethod1::speedCb(const control_msgs::GetECUReport msg) {

    if (this->super_points_.on_off.p_verbose) {
        ROS_INFO_STREAM("CS: speedCb");
    }
    this->getECUReport_ = msg;
    this->super_points_.cur_speed.speed = (this->getECUReport_.motor_n_now_L + this->getECUReport_.motor_n_now_R) / 2 / 3.6 / 148.5;

    if (fabs(this->super_points_.cur_speed.speed) <= 10 / 3.6 / 148.5) {
        this->super_points_.cur_direction = gear_type::N;
    } else {
        if (this->super_points_.cur_speed.speed > 0) {
            this->super_points_.cur_direction = gear_type::D1;
        } else {
            this->super_points_.cur_direction = gear_type::R;
        }
    }
    this->super_points_.cur_speed.speed = fabs(this->super_points_.cur_speed.speed);

    this->super_points_.cur_speed.seq.erase(this->super_points_.cur_speed.seq.begin());
    this->super_points_.cur_speed.seq.emplace_back(this->super_points_.cur_speed.speed);

    this->super_points_.cur_acc.acc = this->super_points_.cur_speed.seq.back() - this->super_points_.cur_speed.seq[0];

    double_t tmp_radius_of_curvature = 100000;
    double_t width_between_wheels = 2.5;
    if (fabs(msg.motor_n_now_L - msg.motor_n_now_R) > 0.001) {
        tmp_radius_of_curvature = fabs((msg.motor_n_now_L + msg.motor_n_now_R) / (msg.motor_n_now_L - msg.motor_n_now_R) * width_between_wheels * 0.5);
    }
    this->super_points_.cur_steer.steer = tmp_radius_of_curvature;

    this->super_points_.cur_steer.seq.erase(this->super_points_.cur_steer.seq.begin());
    this->super_points_.cur_steer.seq.emplace_back(this->super_points_.cur_steer.steer);
    this->super_points_.cur_steer.fabs_max = fabs(this->super_points_.cur_steer.steer);
    for (const auto & tmp_steer : this->super_points_.cur_steer.seq) {
        if (this->super_points_.cur_steer.fabs_max > fabs(tmp_steer)) {
            this->super_points_.cur_steer.fabs_max = fabs(tmp_steer);
        }
    }
    this->super_points_.msg_udt.update.items.speed = true;
}
#endif

void CSMethod1::decide_running_mode() {
    this->super_points_.running_mode = running_mode_type::unknown;

    if ((this->trajectory_.header.frame_id == "base_link") && (this->trajectory_.points[0].forward == 1)) {
        this->super_points_.running_mode = running_mode_type::general_ahead;
    }
    if ((this->trajectory_.header.frame_id == "base_link") && (this->trajectory_.points[0].forward == 0)) {
        this->super_points_.running_mode = running_mode_type::general_retreat;
    }
    if ((this->trajectory_.header.frame_id == "base_link") && (this->trajectory_.points[0].forward == 1) && (this->super_points_.dynamic_collision.collision)) {
        this->super_points_.running_mode = running_mode_type::dynamic_collision_ahead;
    }
    if ((this->trajectory_.header.frame_id == "base_link") && (this->trajectory_.points[0].forward == 0) && (this->super_points_.dynamic_collision.collision)) {
        this->super_points_.running_mode = running_mode_type::dynamic_collision_ahead;
    }
    if ((this->trajectory_.header.frame_id == "base_link") && (this->trajectory_.points[0].forward == 1) && (this->way_.foggy_area == 1)) {
        this->super_points_.running_mode = running_mode_type::foggy_ahead;
    }
    if ((this->trajectory_.header.frame_id == "base_link") && (this->trajectory_.points[0].forward == 0) && (this->way_.foggy_area == 1)) {
        this->super_points_.running_mode = running_mode_type::foggy_retreat;
    }
    if ((this->trajectory_.header.frame_id == "base_link") && (this->trajectory_.points[0].forward == 1) && (this->way_.task_area == "patrol_area")) {
        this->super_points_.running_mode = running_mode_type::inspect_ahead;
    }
    if ((this->trajectory_.header.frame_id == "base_link") && (this->trajectory_.points[0].forward == 0) && (this->way_.task_area == "patrol_area")) {
        this->super_points_.running_mode = running_mode_type::inspect_retreat;
    }
    if ((this->trajectory_.header.frame_id == "search_base_link") && (this->trajectory_.points[0].forward == 1)) {
        this->super_points_.running_mode = running_mode_type::search_ahead;
    }
    if ((this->trajectory_.header.frame_id == "search_base_link") && (this->trajectory_.points[0].forward == 0)) {
        this->super_points_.running_mode = running_mode_type::search_retreat;
    }
    if ((this->trajectory_.header.frame_id == "search_base_link") && (this->super_points_.warn_diff_direction)) {
        this->super_points_.running_mode = running_mode_type::search_direction_diff;
    }

    ROS_INFO_STREAM("CS: running mode: " << this->running_mode_strings_[this->super_points_.running_mode]);
}

void CSMethod1::find_first_min_v() {
    bool tmp_find = false;
    for (count_type i = 1; i < this->super_points_.point_num - 1; ++i) {
        if ((this->super_points_.v_constrained[i] < this->super_points_.v_constrained[i + 1]) && (this->super_points_.v_constrained[i] < this->super_points_.v_constrained[i - 1])) {
            tmp_find = true;
            this->speed_ctrl_.v_max = this->super_points_.v_constrained[i];
        }
    }
    if (!tmp_find) {
        this->speed_ctrl_.v_max = this->super_points_.v_constrained.back();
    }
}

void CSMethod1::time_check() {
    if ((ros::Time::now().toSec() * 1000 - this->super_points_.time_check.rc_way_time) > 4000) {
        this->way_.vel_limit = 20;
        this->way_.task_area = "default";
        this->way_.is_forward = 1;
        this->way_.foggy_area = 0;
        this->way_.lawn_area = 0;
        this->way_.points.clear();
        ROS_INFO_STREAM("CS: roadnet info not receive, set to default");
    }
}

void CSMethod1::limit_segmentRoad() {
    if (this->way_.points.size() < 5) {
        this->super_points_.segmentRoad.v_limit = 20;
        ROS_INFO_STREAM("CS: receive roadnet points num less" << this->way_.points.size());
        return;
    }
    if (this->way_.is_forward == 0) {
        ROS_INFO_STREAM("CS: receive roadnet retreat");
        return;
    }

    this->super_points_.segmentRoad.x_points.clear();
    this->super_points_.segmentRoad.y_points.clear();
    for (count_type i = 0; i < this->way_.points.size(); ++i) {
        this->super_points_.segmentRoad.x_points.emplace_back(this->way_.points[i].point.x - this->way_.points[0].point.x);
        this->super_points_.segmentRoad.y_points.emplace_back(this->way_.points[i].point.y - this->way_.points[0].point.y);
    }

    this->point_attributes_params_segmentRoad_.curvature_max = 100;
    this->point_attributes_params_segmentRoad_.min_size = 3;
    this->point_attributes_params_segmentRoad_.max_size = 300;
    this->point_attributes_params_segmentRoad_.curv_zero_redundancy = 0.4;
    this->point_attributes_params_segmentRoad_.min_total_length = 10;
    this->point_attributes_params_segmentRoad_.half_s_curv_cal = 1;

    this->pointsAttributes_segmentRoad_.calculate(this->point_attributes_params_segmentRoad_, this->super_points_.segmentRoad.x_points, this->super_points_.segmentRoad.y_points);
    if (this->pointsAttributes_segmentRoad_.error.update.result != 0) {
        if (this->pointsAttributes_segmentRoad_.error.update.items.s_damn_short) {
            ROS_WARN_STREAM("CS: roadnet received is too short");
        } else {
            ROS_WARN_STREAM("CS: roadnet limit set to default");
        }
        this->super_points_.segmentRoad.v_limit = 20;
        return;
    }

    this->super_points_.segmentRoad.consider_points_num = count_type(fabs(this->getECUReport_.speed.velocity.linear.x) * 5.0);
    this->super_points_.segmentRoad.consider_points_num = count_type(std::min(((unsigned long)(this->super_points_.segmentRoad.consider_points_num)), this->way_.points.size()));
    if (this->super_points_.segmentRoad.consider_points_num < 5) {
        this->super_points_.segmentRoad.v_limit = 20;
        return;
    }
    this->super_points_.segmentRoad.result_curvature_max = 0.001;
    for (count_type i = 0; i < this->super_points_.segmentRoad.consider_points_num; ++i) {
        if (this->super_points_.segmentRoad.result_curvature_max < this->pointsAttributes_segmentRoad_.curv_cal.curv_final[i]) {
            this->super_points_.segmentRoad.result_curvature_max = this->pointsAttributes_segmentRoad_.curv_cal.curv_final[i];
        }
    }
    this->super_points_.segmentRoad.v_limit = sqrt(1.0 / this->super_points_.segmentRoad.result_curvature_max);

    ROS_WARN_STREAM("test roadnet info starts");
    for (count_type i = 0; i < this->pointsAttributes_segmentRoad_.curv_cal.curv_final.size(); ++i) {
        ROS_INFO_STREAM("CS: roadnet: " << i <<": "<< this->pointsAttributes_segmentRoad_.curv_cal.curv_final[i]);
    }
    ROS_INFO_STREAM("consider points num: " << this->super_points_.segmentRoad.consider_points_num);
    ROS_INFO_STREAM("v limit: " << this->super_points_.segmentRoad.v_limit);
    ROS_INFO_STREAM("curvature max : " << this->super_points_.segmentRoad.result_curvature_max);
    ROS_WARN_STREAM("test roadnet info ends");
}

}
