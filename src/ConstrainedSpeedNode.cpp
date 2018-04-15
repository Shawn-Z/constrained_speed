#include <nodelet/loader.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>

int main(int argc, char **argv) {
    ros::init(argc, argv, "constrained_speed");
    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();
    nodelet.load(nodelet_name, "constrained_speed/constrained_speed_nodelet", remap,
                 nargv);
    ros::spin();
    return 0;
}
