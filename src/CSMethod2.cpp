#include "CSMethod2.hpp"
namespace constrained_speed {

CSMethod2::CSMethod2(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
    : reconfigSrv_{private_node_handle}, params_{private_node_handle} {

    this->nh_ = node_handle;
    this->private_nh_ = private_node_handle;

    glogInit();

    this->reconfigSrv_.setCallback(boost::bind(&CSMethod2::reconfigureRequest, this, _1, _2));
    this->params_.fromParamServer();

    paramsInit();
    setHandle();

    //// todo SETTING. modify topic name as you need
    this->roadnet_sub_ = this->nh_.subscribe<lanelet_map_msgs::Way>(this->yaml_params_.road_net_topic, 1, &CSMethod2::roadnetCb, this);
    this->path_sub_ = this->nh_.subscribe<plan2control_msgs::Trajectory>("/global_path/traj_plan", 1, &CSMethod2::pathCb, this);
    this->steer_cmd_sub_ = this->nh_.subscribe<three_one_msgs::ControlSteer>("/steer_cmd", 1, &CSMethod2::steerCmdCb, this);
    this->three_one_ecu_sub_ = this->nh_.subscribe<three_one_msgs::Report>("/ecudatareport", 1, &CSMethod2::three_one_ecuCb, this);

    if (strcmp(ISSUE_MODE, "cycle") == 0) {
        this->process_timer_ = this->nh_.createTimer(ros::Duration(PROCESS_PERIOD), boost::bind(&CSMethod2::planning, this));
    }
    this->check_timer_ = this->nh_.createTimer(ros::Duration(TIME_CHECK_PERIOD), boost::bind(&CSMethod2::time_check, this));
}

void CSMethod2::glogInit() {
    this->sLog_.init("constrained_speed", "log_constrained_speed", google::ERROR);
    LOG_INFO << "program start";
    LOG_WARN << "program start";
    LOG_ERROR << "program start";
}

void CSMethod2::paramsInit() {
    bool tmp_result = true;
    tmp_result &= this->private_nh_.getParam("acc_lat_max", this->yaml_params_.acc_lat);
    tmp_result &= this->private_nh_.getParam("acc_lon_max", this->yaml_params_.acc_max);
    tmp_result &= this->private_nh_.getParam("dec_lon_max", this->yaml_params_.acc_min);
    tmp_result &= this->private_nh_.getParam("safe_dec", this->yaml_params_.safe_dec);
    tmp_result &= this->private_nh_.getParam("v_max", this->yaml_params_.v_max);
    tmp_result &= this->private_nh_.getParam("road_net_topic", this->yaml_params_.road_net_topic);
    if (!tmp_result) {
        ROS_WARN_STREAM("CS: param not retrieved");
        ros::shutdown();
    }
    this->yaml_params_.acc_lat = fabs(this->yaml_params_.acc_lat);
    this->yaml_params_.acc_max = fabs(this->yaml_params_.acc_max);
    this->yaml_params_.acc_min = -fabs(this->yaml_params_.acc_min);
    this->yaml_params_.safe_dec = -fabs(this->yaml_params_.safe_dec);
    this->yaml_params_.v_max = fabs(this->yaml_params_.v_max);
}

void CSMethod2::planning() {
    if (this->no_msg_) {
        return;
    }
    if (!this->pointsPretreat()) {
        return;
    }
    if (!this->pointsAttributes()) {
        return;
    }
    this->v_.assign(this->points_size_, 100.0);
    this->v_[0] = this->cur_speed_;
    if (!this->setVMax()) {
        return;
    }
    if (!this->limitLatAcc()) {
        return;
    }
    if (!this->blindHandle()) {
        return;
    }
    if (!this->limitBySteer()) {
        return;
    }
    if (!this->durex()) {
        return;
    }
    if (!this->smoothSpeed()) {
        return;
    }
    if (!this->calTime()) {
        return;
    }
    if (!this->calAcc()) {
        return;
    }
//    if (!this->calJerk()) {
//        return;
//    }
    if (!this->publish()) {
        return;
    }
}

bool CSMethod2::pointsPretreat() {
    //// todo SETTING. modify value below as you need
    double_t min_total_length = 0.1;

    if (this->trajectory_.points.size() < 2) {
        LOG_ERROR << "CS: receive points size: " << this->trajectory_.points.size();
        return false;
    }
    if (this->trajectory_.points.size() > 1000) {
        LOG_ERROR << "CS: receive points size: " << this->trajectory_.points.size();
        return false;
    }

    this->x_points_.clear();
    this->y_points_.clear();
    this->points_size_ = this->trajectory_.points.size();
    size_t direction_index = 0;
    for (size_t i = 0; i < this->points_size_; ++i) {
        this->x_points_.emplace_back(this->trajectory_.points[i].position.x);
        this->y_points_.emplace_back(this->trajectory_.points[i].position.y);
        if (this->trajectory_.points[i].forward != this->trajectory_.points[0].forward) {
            break;
        }
        direction_index = i;
    }
    shawn::SPoints sPoints;
    this->intervals_arc_lengths_ = sPoints.calculate_intervals_arc_lengths(this->x_points_, this->y_points_);
    this->intervals_ = this->intervals_arc_lengths_[0];
    this->arc_lengths_ = this->intervals_arc_lengths_[1];

    if ((this->arc_lengths_.back() < min_total_length) || (direction_index == 0)) {
        this->x_points_.clear();
        this->y_points_.clear();
        ++direction_index;
        for (size_t i = direction_index; i < this->points_size_; ++i) {
            this->x_points_.emplace_back(this->trajectory_.points[i].position.x);
            this->y_points_.emplace_back(this->trajectory_.points[i].position.y);
            if (this->trajectory_.points[i].forward != this->trajectory_.points[direction_index].forward) {
                break;
            }
            direction_index = i;
        }
        this->intervals_arc_lengths_ = sPoints.calculate_intervals_arc_lengths(this->x_points_, this->y_points_);
        this->intervals_ = this->intervals_arc_lengths_[0];
        this->arc_lengths_ = this->intervals_arc_lengths_[1];
        if (this->intervals_arc_lengths_.empty()) {
            LOG_ERROR << "CS: receive points illegal";
            return false;
        }
        if (this->arc_lengths_.back() < min_total_length) {
            LOG_ERROR << "CS: length of path: " << this->arc_lengths_.back();
            return false;
        }
        if (direction_index != (this->trajectory_.points.size() - 1)) {
            LOG_ERROR << "CS: path change direction more than once";
        }
    }

    this->direction_ = this->trajectory_.points[direction_index].forward? direction::forward : direction::backward;

    //// todo xi shu jian cha

    this->points_size_ = this->x_points_.size();
    return true;
}

bool CSMethod2::pointsAttributes() {
    //// todo SETTING. modify value below as you need
    double_t waist_length = 1.0;
    double_t concerned_curvature = 0.06;
    double_t max_unsigned_curvature = 1.25;

    shawn::SThreePointsCurvature sCurvature;
    this->curvatures_ = sCurvature.speedPlanningCurvature(this->x_points_,
                                                          this->y_points_,
                                                          this->arc_lengths_,
                                                          this->intervals_,
                                                          waist_length,
                                                          concerned_curvature,
                                                          max_unsigned_curvature);
    if (this->curvatures_.size() != this->points_size_) {
        ROS_ERROR_STREAM("empty curvatures");
        this->curvatures_.assign(this->points_size_, max_unsigned_curvature);
    }
    return true;
}

bool CSMethod2::setVMax() {
    //// todo SETTING. modify value below as you need
    double_t jerk_slow_down_forward = 2.0;
    double_t jerk_slow_down_middle = 0.1;
    double_t slow_down_diff = 0.001;
    size_t slow_down_cycles = 100;

    double_t v_max = 100.0;
    v_max = std::min(v_max, this->yaml_params_.v_max);
//    v_max = std::min(v_max, this->way_.vel_limit);
    if (this->direction_ == direction::backward) {
        v_max = std::min(v_max, 1.5);
    }
    if (this->trajectory_.header.frame_id == "search_base_link") {
        v_max = std::min(v_max, this->trajectory_.points[0].velocity.linear.x);
    }
    if (this->way_.task_area == "patrol_area") {
        v_max = std::min(v_max, 1.5);
    }
    if (this->way_.foggy_area == 1) {
        v_max = std::min(v_max, 1.5);
    }

    this->v_.assign(this->points_size_, v_max);
    this->v_[0] = this->cur_speed_;
    if (this->v_[0] > v_max) {
        shawn::SJerk sJerk;
        sJerk.smooth_slow_down(this->v_,
                               this->intervals_,
                               v_max,
                               std::min(this->cur_acc_, 0.0),
                               this->yaml_params_.acc_max,
                               this->yaml_params_.acc_min,
                               jerk_slow_down_forward,
                               jerk_slow_down_middle,
                               slow_down_diff,
                               slow_down_cycles);
    }
    return true;
}

bool CSMethod2::limitLatAcc() {
    double_t zero_curvature = this->yaml_params_.acc_lat / 10000;
    //// todo make small curvature to zero
    zero_curvature = 0.055;
    for (size_t i = 1; i < points_size_; ++i) {
        if (this->curvatures_[i] > zero_curvature) {
            this->v_[i] = std::min(this->v_[i], pow(this->yaml_params_.acc_lat / this->curvatures_[i], 0.5));
        }
    }
    return true;
}

bool CSMethod2::smoothSpeed() {
    //// todo SETTING. modify value below as you need
    double_t jerk_forward_pass = 4.0;

    double_t jerk_backward_pass = 200.0;
    double_t slide_dec = -0.2;
    double_t a_end = 0.0;
    double_t v_begin_threshold = 0.01;
    bool remove_dec = false;
    double_t remove_dec_value = -0.2;
    double_t remove_dec_limit_v = std::max(this->cur_speed_, 2.0);

    double_t jerk_middle_pass = 200.0;
    double_t middle_cycle_diff = 0.001;
    size_t middle_cycles = 100;

    shawn::SJerk sJerk;
    sJerk.forward_pass_concave(this->v_,
                               this->intervals_,
                               jerk_forward_pass,
                               this->yaml_params_.acc_max,
                               this->yaml_params_.acc_min,
                               this->cur_acc_);

    std::vector<double_t> origin_v = this->v_;
    bool tmp_result = false;
//    if (!tmp_result) {
//        this->v_ = origin_v;
//        tmp_result = sJerk.backward_pass_concave(this->v_,
//                                                 this->intervals_,
//                                                 jerk_backward_pass,
//                                                 this->yaml_params_.acc_max,
//                                                 slide_dec,
//                                                 a_end,
//                                                 v_begin_threshold,
//                                                 remove_dec,
//                                                 remove_dec_value,
//                                                 remove_dec_limit_v);
//    }
    if (!tmp_result) {
        this->v_ = origin_v;
        tmp_result = sJerk.backward_pass_concave(this->v_,
                                                 this->intervals_,
                                                 jerk_backward_pass,
                                                 this->yaml_params_.acc_max,
                                                 this->yaml_params_.acc_min,
                                                 a_end,
                                                 v_begin_threshold,
                                                 remove_dec,
                                                 remove_dec_value,
                                                 remove_dec_limit_v);
    }
    if (!tmp_result) {
        this->v_ = origin_v;
        tmp_result = sJerk.backward_pass_concave(this->v_,
                                                 this->intervals_,
                                                 DBL_MAX,
                                                 this->yaml_params_.acc_max,
                                                 this->yaml_params_.acc_min,
                                                 a_end,
                                                 v_begin_threshold,
                                                 remove_dec,
                                                 remove_dec_value,
                                                 remove_dec_limit_v);
    }
    if (!tmp_result) {
        this->v_ = origin_v;
        tmp_result = sJerk.backward_pass_concave(this->v_,
                                                 this->intervals_,
                                                 DBL_MAX,
                                                 this->yaml_params_.acc_max,
                                                 this->yaml_params_.safe_dec,
                                                 a_end,
                                                 v_begin_threshold,
                                                 remove_dec,
                                                 remove_dec_value,
                                                 remove_dec_limit_v);
    }

    sJerk.middle_pass_convex_cycle(this->v_, this->intervals_, jerk_middle_pass, middle_cycle_diff, middle_cycles);

    if (!tmp_result) {
        LOG_ERROR << "CS: backward pass powerless";
    }
    return true;
}

bool CSMethod2::blindHandle() {
    //// todo SETTING. modify value below as you need
    bool blind = true;
    bool use_blind_time = true;
    double_t blind_time = 0.5;
    double_t blind_area = 10.0;
    double_t blind_dec = -0.2;
    double_t blind_limit_v = 2.0;
    bool blind_reactive = true;

    if (!blind) {
        return true;
    }

    if (this->cur_speed_ < blind_limit_v) {
        return true;
    }

    if (use_blind_time) {
        blind_time = std::min(blind_time, this->cur_speed_ / fabs(blind_dec));
        blind_area = this->cur_speed_ * blind_time - 0.5 * fabs(blind_dec) * pow(blind_time, 2.0);
    }
    size_t end_index = this->points_size_;
    for (size_t i = 1; i < this->points_size_; ++i) {
        if (this->arc_lengths_[i] > blind_area) {
            end_index = i;
            break;
        }
    }
    double_t tmp_speed = this->v_[0];
    shawn::SJerk sJerk;
    double_t tmp_v_min = this->v_[0];
    for (size_t j = 1; j <= end_index; ++j) {
        tmp_v_min = std::min(tmp_v_min, this->v_[j]);
        tmp_speed = sJerk.calculate_next_speed(tmp_speed, 0, this->intervals_[j], 0, -fabs(blind_dec), -fabs(blind_dec), false);
        this->v_[j] = std::max(this->v_[j], tmp_speed);
    }
    if (blind_reactive) {
        this->v_[end_index + 1] = std::min(this->v_[end_index + 1], tmp_v_min);
    }
    return true;
}

bool CSMethod2::durex() {
    //// todo SETTING. modify value below as you need
    //// todo SETTING. remember adjust safe redundancy in different mode
    double_t general_space_front = 2.0;
    double_t general_space_rear = 2.5;
    double_t search_space_front = 3.8;
    double_t search_space_rear = 1.2;

    double_t safe_redundancy = 1.0;
    safe_redundancy += general_space_front;
    double_t dec_delay = 0.3;

    double_t at = fabs(this->yaml_params_.safe_dec * dec_delay);
    double_t effective_length = std::max((this->arc_lengths_.back() - safe_redundancy), 0.0);
    double_t v_durex = sqrt(pow(at, 2.0) + 2.0 * fabs(this->yaml_params_.safe_dec) * effective_length) - at;
    for (size_t i = 1; i < this->points_size_; ++i) {
        this->v_[i] = std::min(this->v_[i], v_durex);
    }
    return true;
}

void CSMethod2::time_check() {
    //// todo SETTING. modify check range as you need
    bool path_till_now_check = this->sub_times_.checkSingleTimestampTillNow(this->path_sub_handle_, -1, 150);
    bool ecu_till_now_check = this->sub_times_.checkSingleTimestampTillNow(this->ecu_sub_handle_, -1, 100);
    bool roadnet_till_now_check = this->sub_times_.checkSingleTimestampTillNow(this->roadnet_sub_handle_, -1, 4000);
    bool steer_cmd_till_now_check = this->sub_times_.checkSingleTimestampTillNow(this->steer_cmd_sub_handle_, -1, 100);

    this->no_msg_ = (!path_till_now_check) || (!ecu_till_now_check);
    if (!path_till_now_check) {
        LOG_ERROR << "CS: path not update";
    }
    if (!ecu_till_now_check) {
        LOG_ERROR << "CS: ecu not update";
    }
    if (!roadnet_till_now_check) {
        this->way_.vel_limit = 100.0;
        this->way_.task_area = "none";
        this->way_.foggy_area = 0;
    }
    if (!steer_cmd_till_now_check) {
        this->control_steer_.curvature = 0.0;
    }
}

void CSMethod2::pathCb(const plan2control_msgs::Trajectory msg) {
    this->trajectory_ = msg;
    this->sub_times_.pushTimestamp(this->path_sub_handle_);
    if (strcmp(ISSUE_MODE, "direct") == 0) {
        this->planning();
    }
}

void CSMethod2::setHandle() {
    this->path_sub_handle_ = this->sub_times_.time_handle.newHandle("check the period of path");
    this->ecu_sub_handle_ = this->sub_times_.time_handle.newHandle("check the period of ecu");
    this->roadnet_sub_handle_ = this->sub_times_.time_handle.newHandle("check the period of roadnet");
    this->steer_cmd_sub_handle_ = this->sub_times_.time_handle.newHandle("check the period of steer cmd");
}

void CSMethod2::roadnetCb(const lanelet_map_msgs::Way msg) {
    this->way_ = msg;
    this->sub_times_.pushTimestamp(this->roadnet_sub_handle_);
}

void CSMethod2::reconfigureRequest(cs_method2Config &config, uint32_t level) {
}

void CSMethod2::steerCmdCb(const three_one_msgs::ControlSteer msg) {
   this->control_steer_ = msg;
   this->sub_times_.pushTimestamp(this->steer_cmd_sub_handle_);
}

bool CSMethod2::limitBySteer() {
    this->control_steer_.curvature = std::max<float>(0.0001, fabs(this->control_steer_.curvature));
    //// todo adpat for three_one lateral control
    this->control_steer_.curvature /= 3.0;
    double_t v = sqrt(fabs(this->yaml_params_.acc_lat / this->control_steer_.curvature));
    for (size_t i = 1; i < this->points_size_; ++i) {
        this->v_[i] = std::min(this->v_[i], v);
    }
    return true;
}

bool CSMethod2::calTime() {
    //// todo SETTING. modify value below as you need
    double_t zero_speed = 0.01;

    this->time_.clear();
    this->time_.emplace_back(0.0);
    this->time_interval_.clear();
    this->time_interval_.emplace_back(0.0);
    bool speed_zero = true;
    for (size_t i = 1; i < this->points_size_; ++i) {
        speed_zero &= ((this->v_[i] < zero_speed) && (this->v_[i - 1] < zero_speed));
        if (speed_zero) {
            this->time_interval_.emplace_back(DBL_MAX);
            this->time_.emplace_back(DBL_MAX);
        } else {
            this->time_interval_.emplace_back(2.0 * this->intervals_[i] / (this->v_[i - 1] + this->v_[i]));
            this->time_.emplace_back(this->time_[i - 1] + this->time_interval_[i]);
        }
    }
    return true;
}

bool CSMethod2::calAcc() {
    this->acc_.clear();
    this->acc_.emplace_back(this->cur_acc_);
    for (size_t i = 1; i < this->points_size_; ++i) {
        this->acc_.emplace_back((pow(this->v_[i], 2.0) - pow(this->v_[i - 1], 2.0)) * 0.5 / this->intervals_[i]);
    }
    return true;
}

bool CSMethod2::calJerk() {
    this->jerk_.clear();
    this->jerk_.emplace_back(0.0);
    for (size_t i = 1; i < this->points_size_; ++i) {
        this->jerk_.emplace_back((this->acc_[i] - this->acc_[i - 1]) / this->time_interval_[i]);
    }
    return true;
}

bool CSMethod2::publish() {
    //// todo SETTING. modify value below as you need
    double_t acc_delay = 0.3;
    double_t dec_delay = 2.0;
//    static Toyota toyota;
//    toyota_issue issue_result = toyota.publish(this->nh_, this->time_, this->v_, this->acc_, acc_delay, dec_delay, (this->direction_ == direction::forward));

    static ThreeOnePublish threeOnePublish;
    three_one_issue issue_result = threeOnePublish.publish(this->nh_, this->time_, this->v_, this->acc_, acc_delay, (this->direction_ == direction::forward));

    std::vector<double_t> tmp_issue_result;
    tmp_issue_result.emplace_back(issue_result.v);
    tmp_issue_result.emplace_back(issue_result.acc);
    additionPublish(tmp_issue_result);

    return true;
}

void CSMethod2::additionPublish(std::vector<double_t> issue) {
    static speed_debug_msgs::speed_debug speed_debug;
    static ros::Publisher debug_pub = this->nh_.advertise<speed_debug_msgs::speed_debug>("/speed_debug", 1);
    speed_debug.points.resize(this->points_size_);
    for (size_t i = 0; i < this->points_size_; ++i) {
        speed_debug.points[i].v.v_constrained = this->v_[i];
        speed_debug.points[i].curv.curv_final = fabs(this->curvatures_[i]);
        speed_debug.points[i].time.time = this->time_[i];
        speed_debug.points[i].x = this->x_points_[i];
        speed_debug.points[i].y = this->y_points_[i];
        speed_debug.points[i].s = this->arc_lengths_[i];
        speed_debug.points[i].acc = this->acc_[i];
        speed_debug.points[0].curv.curv_final = std::max(speed_debug.points[0].curv.curv_final, speed_debug.points[i].curv.curv_final);
    }
    speed_debug.issue.v = issue[0];
    speed_debug.issue.acc = issue[1];
    speed_debug.pub_ros_time = ros::Time::now().toSec();
    debug_pub.publish(speed_debug);
    ROS_INFO_STREAM(speed_debug.points[0].curv.curv_final);
}

void CSMethod2::three_one_ecuCb(const three_one_msgs::Report msg) {
    static std::vector<double_t> speed_seq(50, 0.0);
    speed_seq.erase(speed_seq.begin());
    speed_seq.emplace_back(msg.motion.vehicle_speed);
    this->cur_acc_ = speed_seq.back() - speed_seq[0];
    this->cur_speed_ = msg.motion.vehicle_speed;
    this->sub_times_.pushTimestamp(this->ecu_sub_handle_);
}

}