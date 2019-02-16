#ifndef CONSTRAINED_SPEED_DEFINES_HPP
#define CONSTRAINED_SPEED_DEFINES_HPP

#define LOG_INFO LOG(INFO)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define LOG_WARN LOG(WARNING)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define LOG_ERROR LOG(ERROR)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define LOG_FATAL LOG(FATAL)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define FIXED std::setiosflags(std::ios::fixed)<<

#define PROCESS_PERIOD 0.1
#define ISSUE_MODE "direct" // direct cycle
#define TIME_CHECK_PERIOD 0.2

#endif //CONSTRAINED_SPEED_DEFINES_HPP