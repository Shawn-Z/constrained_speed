#include "CSMethod2.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "constrained_speed");
    constrained_speed::CSMethod2 csMethod2(ros::NodeHandle(), ros::NodeHandle("~"));
    ros::spin();
    return 0;
}