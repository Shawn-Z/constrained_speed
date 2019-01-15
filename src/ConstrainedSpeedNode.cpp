#include "CSMethod1.hpp"


int main(int argc, char **argv) {
    ros::init(argc, argv, "constrained_speed");
    constrained_speed::CSMethod1 csMethod1(ros::NodeHandle(), ros::NodeHandle("~"));
    ros::spin();
    return 0;
}
